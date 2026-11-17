"""ROS 2 adapter for AegisNav mission assessment and path publication.

By default, this node publishes a path and structured decision for inspection.
Optional Nav2 FollowPath execution is explicitly disabled until the integrator
has verified map frames, localization, controller parameters and collision
behavior in their own simulation.
"""
from __future__ import annotations

import json
import math
import threading
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, String
from aegisnav_msgs.msg import HazardForecast

from action_msgs.msg import GoalStatus
from aegisnav_core.execution import (
    CommandKind, ExecCommand, ExecState, ExecutionController, GoalOutcome, keep_active_route, remaining_route, stale_inputs,
)
from aegisnav_core.mission import ConfidenceHysteresis, MissionPolicy, MissionSafetySupervisor
from aegisnav_core.models import Cell, PlannerConfig, RoutePlan
from aegisnav_core.path_geometry import centred_waypoints, densify
from aegisnav_core.planning import RiskAwareAStar, evaluate_route
from aegisnav_core.scenarios import scenario_by_name
from .ros_grid import apply_scenario_parameters, grid_from_occupancy_grid, risk_frames_from_message, snap_to_free_cell, world_to_cell


# map_server publishes the static map once, latched; late subscribers need transient-local.
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class RiskPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_risk_planner")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("forecast_topic", "/aegisnav/hazard_forecast")
        # "tf": pose = map->robot_base_frame transform, stamped by the transform (refreshes while
        # stationary, so staleness is meaningful). "topic": PoseWithCovarianceStamped on pose_topic
        # (AMCL only publishes after the robot moves; use for manual tests).
        self.declare_parameter("pose_source", "tf")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        # Freshness of the TF pose is the age of odom->base (continuous); AMCL's map->odom
        # is only refreshed when the filter updates, so it is not a liveness signal.
        self.declare_parameter("odom_frame", "odom")
        # Keep the active route unless a candidate is at least this fraction cheaper, in [0, 1).
        self.declare_parameter("replan_min_improvement", 0.15)
        # Metres. A pose this close to a free cell (e.g. AMCL noise beside a wall) plans from
        # that cell; a pose deeper inside an obstacle cell still fails closed.
        self.declare_parameter("start_snap_tolerance_m", 0.25)
        # Metres between published path poses (route is centred, then densified).
        self.declare_parameter("path_spacing_m", 0.1)
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("confidence_topic", "/aegisnav/perception_confidence")
        self.declare_parameter("path_topic", "/aegisnav/planned_path")
        self.declare_parameter("decision_topic", "/aegisnav/decision")
        # Non-empty: base and refuges come from aegisnav_core.scenarios (overrides the two lists below).
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("base_cell_xy", [2, 2])
        self.declare_parameter("refuge_cells_xy", [2, 12, 21, 2, 6, 12])
        self.declare_parameter("hazard_weight", 12.0)
        self.declare_parameter("uncertainty_weight", 2.5)
        self.declare_parameter("hard_risk_threshold", 0.92)
        self.declare_parameter("nominal_speed_mps", 0.35)
        self.declare_parameter("wall_clearance_penalty_m", 0.3)
        self.declare_parameter("wall_clearance_cells", 1)
        self.declare_parameter("straight_near_obstacles", False)
        self.declare_parameter("inflate_obstacles_cells", 0)
        self.declare_parameter("minimum_continue_confidence", 0.55)
        self.declare_parameter("critical_confidence", 0.30)
        # After degraded confidence, continue only once it recovers to this (hysteresis, no flapping).
        self.declare_parameter("resume_confidence", 0.65)
        self.declare_parameter("max_outbound_exposure", 18.0)
        self.declare_parameter("max_return_exposure", 18.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("execute_with_follow_path", False)
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        # Seconds; pose and forecast older than this make the cycle fail closed.
        self.declare_parameter("slow_cycle_warn_s", 1.0)  # log cycles slower than this (0 = log every cycle)
        self.declare_parameter("max_input_age_s", 3.0)

        name = str(self.get_parameter("scenario_name").value)
        if name:
            scn = scenario_by_name(name)
            apply_scenario_parameters(self, name, {
                "base_cell_xy": [scn.base.x, scn.base.y],
                "refuge_cells_xy": [v for c in scn.refuges for v in (c.x, c.y)],
            })
        self._lock = threading.RLock()
        self._map_msg: OccupancyGrid | None = None
        self._grid = None
        self._forecast_msg: HazardForecast | None = None
        self._pose_msg: PoseWithCovarianceStamped | None = None
        self._goal_msg: PoseStamped | None = None
        self._confidence = 1.0
        self._dirty = False
        self._active_follow_handle = None
        self._exec = ExecutionController()
        self._last_decision: tuple[str, str] | None = None
        self._exec_lock = threading.RLock()
        self._stamps_s: dict[str, float | None] = {"pose": None, "forecast": None}
        self._planner = RiskAwareAStar()
        self._supervisor = self._build_supervisor()
        self._min_improvement = float(self.get_parameter("replan_min_improvement").value)
        if not 0.0 <= self._min_improvement < 1.0:
            raise ValueError("replan_min_improvement must be in [0, 1)")
        if not float(self.get_parameter("path_spacing_m").value) > 0.0:
            raise ValueError("path_spacing_m must be positive")

        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, MAP_QOS)
        self.create_subscription(HazardForecast, str(self.get_parameter("forecast_topic").value), self._on_forecast, 1)
        pose_source = str(self.get_parameter("pose_source").value)
        if pose_source == "topic":
            self.create_subscription(PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value), self._on_pose, 5)
        elif pose_source == "tf":
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
            self.create_timer(0.1, self._poll_tf)
        else:
            raise ValueError(f"pose_source must be 'tf' or 'topic', got {pose_source!r}")
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self._on_goal, 5)
        self.create_subscription(Float32, str(self.get_parameter("confidence_topic").value), self._on_confidence, 5)
        self._path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 5)
        self._decision_pub = self.create_publisher(String, str(self.get_parameter("decision_topic").value), 10)
        # Execution events actually issued to the controller (JSON): what was sent or cancelled and why.
        self._event_pub = self.create_publisher(String, "/aegisnav/execution_event", 10)
        self._shortest_cache: dict = {}  # (grid, start, target) -> hazard-blind route cells, display only
        self._follow_client = ActionClient(self, FollowPath, str(self.get_parameter("follow_path_action").value))
        self.create_timer(0.25, self._process_if_ready)
        self.get_logger().info("Risk planner ready. It publishes paths by default; motion execution is opt-in.")

    def _build_supervisor(self):
        planner_cfg = PlannerConfig(
            hazard_weight=float(self.get_parameter("hazard_weight").value),
            uncertainty_weight=float(self.get_parameter("uncertainty_weight").value),
            hard_risk_threshold=float(self.get_parameter("hard_risk_threshold").value),
            nominal_speed_mps=float(self.get_parameter("nominal_speed_mps").value),
            wall_clearance_penalty_m=float(self.get_parameter("wall_clearance_penalty_m").value),
            wall_clearance_cells=int(self.get_parameter("wall_clearance_cells").value),
            straight_near_obstacles=bool(self.get_parameter("straight_near_obstacles").value),
            inflate_obstacles_cells=int(self.get_parameter("inflate_obstacles_cells").value),
        )
        self._planner_config = planner_cfg
        policy = MissionPolicy(
            minimum_continue_confidence=float(self.get_parameter("minimum_continue_confidence").value),
            critical_confidence=float(self.get_parameter("critical_confidence").value),
            max_outbound_exposure=float(self.get_parameter("max_outbound_exposure").value),
            max_return_exposure=float(self.get_parameter("max_return_exposure").value),
            planner=planner_cfg,
        )
        self._confidence_latch = ConfidenceHysteresis(
            policy.minimum_continue_confidence, float(self.get_parameter("resume_confidence").value))
        return MissionSafetySupervisor(self._planner, policy)

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            grid = grid_from_occupancy_grid(msg, int(self.get_parameter("occupied_threshold").value))
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        with self._lock:
            self._map_msg, self._grid, self._dirty = msg, grid, True

    def _on_forecast(self, msg: HazardForecast) -> None:
        with self._lock:
            self._forecast_msg, self._dirty = msg, True
            self._stamps_s["forecast"] = self._now_s()

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._pose_msg, self._dirty = msg, True
            self._stamps_s["pose"] = self._now_s()

    def _comparison(self, grid, frames, current, target, selected_plan, time_step_s) -> dict:
        """AegisNav's chosen route vs a distance-only planner that ignores hazards, both scored on the
        current forecast with the same motion/time model. Display data: a live what-if, not a separate run."""
        measure = PlannerConfig(hazard_weight=0.0, uncertainty_weight=0.0, hard_risk_threshold=1.0,
                                nominal_speed_mps=self._planner_config.nominal_speed_mps, wall_clearance_penalty_m=0.0)
        # The hazard-blind route depends only on (grid, start, target); reuse it while it stays legal
        # under the current forecast instead of re-running the time-expanded search every cycle.
        key = (grid, current, target)
        cached = self._shortest_cache.get(key)
        shortest = (evaluate_route(grid, frames, cached, measure, time_step_s=time_step_s)
                    if cached is not None else None)
        if shortest is None or not shortest.success:
            shortest = self._planner.plan(grid, frames, current, target, measure, time_step_s=time_step_s)
            self._shortest_cache = {key: shortest.cells} if shortest.success else {}
        out = {}
        for name, plan in (("aegisnav", selected_plan), ("shortest_path", shortest if shortest.success else None)):
            if plan is None or not plan.success or len(plan.cells) < 2:
                out[name] = None
                continue
            scored = evaluate_route(grid, frames, plan.cells, measure, time_step_s=time_step_s)
            if not scored.success:
                out[name] = None
                continue
            out[name] = {
                "exposure_risk_s": round(scored.hazard_exposure, 2),
                "peak_risk": round(scored.max_risk_encountered, 3),
                "length_m": round(scored.distance_m, 2),
                "crosses_hard_limit": scored.max_risk_encountered >= self._planner_config.hard_risk_threshold,
                "destination": [plan.cells[-1].x, plan.cells[-1].y],
                "route_xy": [[round(v, 2) for v in grid.cell_center_world(c)] for c in plan.cells],
            }
        return out

    def _poll_tf(self) -> None:
        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("robot_base_frame").value)
        try:
            tf = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
            odom_tf = self._tf_buffer.lookup_transform(str(self.get_parameter("odom_frame").value), base_frame, Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warning(f"No {map_frame}->{base_frame} transform: {exc}", throttle_duration_sec=5.0)
            return
        msg = PoseWithCovarianceStamped()
        msg.header = tf.header
        msg.header.frame_id = map_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.orientation = tf.transform.rotation
        stamp_s = odom_tf.header.stamp.sec + odom_tf.header.stamp.nanosec * 1e-9
        # Not marked dirty: planning cycles are paced by the 1 Hz forecast, not the pose rate.
        with self._lock:
            self._pose_msg = msg
            self._stamps_s["pose"] = stamp_s

    def _on_goal(self, msg: PoseStamped) -> None:
        # Goals are re-published periodically; only a changed goal triggers re-planning
        # (the forecast already triggers a cycle every layer step).
        with self._lock:
            old = self._goal_msg
            changed = old is None or old.header.frame_id != msg.header.frame_id or math.hypot(
                old.pose.position.x - msg.pose.position.x, old.pose.position.y - msg.pose.position.y) > 1e-3
            self._goal_msg = msg
            self._dirty = self._dirty or changed
        self.get_logger().debug("Received mission target on %s." % self.get_parameter("goal_topic").value)

    def _on_confidence(self, msg: Float32) -> None:
        value = float(msg.data)
        if not 0.0 <= value <= 1.0:
            self.get_logger().warning("Ignoring perception confidence outside [0, 1].")
            return
        with self._lock:
            self._dirty = self._dirty or abs(value - self._confidence) > 1e-6
            self._confidence = value

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cell_param(self, name: str) -> Cell:
        values = list(self.get_parameter(name).value)
        if len(values) != 2:
            raise ValueError(f"{name} must be [x, y] grid cell coordinates")
        return Cell(int(values[0]), int(values[1]))

    def _refuges(self) -> list[Cell]:
        values = list(self.get_parameter("refuge_cells_xy").value)
        if len(values) % 2:
            raise ValueError("refuge_cells_xy must be a flat even-length list [x0, y0, ...]")
        return [Cell(int(values[i]), int(values[i + 1])) for i in range(0, len(values), 2)]

    def _process_if_ready(self) -> None:
        started = time.monotonic()
        try:
            self._cycle()
        finally:
            elapsed = time.monotonic() - started
            if elapsed > float(self.get_parameter("slow_cycle_warn_s").value):
                self.get_logger().warning(f"Planning cycle took {elapsed:.2f} s (pose is not refreshed meanwhile).")

    def _cycle(self) -> None:
        if str(self.get_parameter("pose_source").value) == "tf":
            self._poll_tf()  # read the freshest pose from the (separately spun) TF buffer
        with self._lock:
            missing = [name for name, item in (("map", self._map_msg), ("forecast", self._forecast_msg), ("pose", self._pose_msg), ("goal", self._goal_msg)) if item is None]
            if missing:
                self.get_logger().warning(f"Waiting for inputs: {missing}", throttle_duration_sec=5.0)
                return
            if not self._dirty:
                return
            self._dirty = False
            stamps = dict(self._stamps_s)
            map_msg, grid, forecast_msg, pose_msg, goal_msg, confidence = (
                self._map_msg, self._grid, self._forecast_msg, self._pose_msg,
                self._goal_msg, self._confidence,
            )
        stale = stale_inputs(self._now_s(), stamps, float(self.get_parameter("max_input_age_s").value))
        if stale:
            self.get_logger().error(f"Stale or missing inputs {stale}; stopping execution and publishing no path.")
            self._publish_no_route(map_msg.header.frame_id or "map", "stale_inputs", f"Inputs {stale} are stale or missing.")
            if bool(self.get_parameter("execute_with_follow_path").value):
                with self._exec_lock:
                    self._dispatch(self._exec.desired(None))
            return
        try:
            map_frame = map_msg.header.frame_id or "map"
            if pose_msg.header.frame_id != map_frame:
                raise ValueError(f"Pose frame {pose_msg.header.frame_id!r} does not match map frame {map_frame!r}")
            if goal_msg.header.frame_id != map_frame:
                raise ValueError(f"Goal frame {goal_msg.header.frame_id!r} does not match map frame {map_frame!r}")
            current = snap_to_free_cell(
                grid, pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y,
                float(self.get_parameter("start_snap_tolerance_m").value),
            )
            target = world_to_cell(grid, goal_msg.pose.position.x, goal_msg.pose.position.y)
            if current is None or target is None:
                raise ValueError(
                    f"Pose ({pose_msg.pose.pose.position.x:.2f}, {pose_msg.pose.pose.position.y:.2f}) or target "
                    f"({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}) is outside the map"
                )
            base = self._cell_param("base_cell_xy")
            refuges = self._refuges()
            frames = risk_frames_from_message(forecast_msg, grid, map_frame)
            time_step_s = max(0.01, float(forecast_msg.time_step_s))
            assessment = self._supervisor.assess(
                grid, frames, current, target, base, refuges, self._confidence_latch.update(confidence),
                time_step_s=time_step_s,
            )
            selected_plan = assessment.execution_route
            if selected_plan is not None and self._exec.state is ExecState.EXECUTING:
                ahead = remaining_route(self._exec.active_cells, current)
                if ahead is not None:
                    active_eval = evaluate_route(grid, frames, ahead, self._planner_config, time_step_s=time_step_s)
                    if keep_active_route(active_eval, selected_plan, min_relative_improvement=self._min_improvement):
                        selected_plan = active_eval  # remaining active route: no cancel/resend
            path_msg = self._path_message(map_msg.header.frame_id or "map", selected_plan, grid)
            if path_msg is not None:
                self._path_pub.publish(path_msg)
            else:
                # Clear stale visual paths so an aborted mission is not shown as active.
                empty_path = Path()
                empty_path.header.frame_id = map_msg.header.frame_id or "map"
                empty_path.header.stamp = self.get_clock().now().to_msg()
                self._path_pub.publish(empty_path)
            # A one-cell route means the robot is in the destination cell: hold (do not send
            # a degenerate goal, and do not cancel a final approach still in progress).
            at_destination = selected_plan is not None and len(selected_plan.cells) <= 1
            comparison = self._comparison(grid, frames, current, target, selected_plan, time_step_s)
            decision_payload = {
                "comparison": comparison,
                "robot_cell": [current.x, current.y],
                # The goal this decision was computed for; consumers must ignore decisions for an older goal.
                "goal_cell": [target.x, target.y],
                "at_destination": at_destination,
                "action": assessment.action.value,
                "reason_code": assessment.reason_code,
                "explanation": assessment.explanation,
                "perception_confidence": round(confidence, 4),
                "safe_return_feasible": assessment.safe_return_feasible,
                "forecast_horizon_sufficient": assessment.forecast_horizon_sufficient,
                "selected_route_success": bool(selected_plan and selected_plan.success),
                "selected_route_distance_m": round(selected_plan.distance_m, 3) if selected_plan and selected_plan.success else None,
                "selected_route_hazard_exposure": round(selected_plan.hazard_exposure, 3) if selected_plan and selected_plan.success else None,
                "grid_dimensions": [grid.width, grid.height],
                "model_notice": "Risk and confidence values are simulation estimates, not certified physical hazard measurements.",
            }
            out = String()
            out.data = json.dumps(decision_payload, sort_keys=True)
            self._decision_pub.publish(out)
            decision_key = (assessment.action.value, assessment.reason_code)
            if decision_key != self._last_decision:
                self.get_logger().info(
                    f"Decision {decision_key[0]} ({decision_key[1]}) at cell ({current.x}, {current.y}), "
                    f"confidence {confidence:.2f}: {assessment.explanation}"
                )
                self._last_decision = decision_key
            if bool(self.get_parameter("execute_with_follow_path").value) and not at_destination:
                with self._exec_lock:
                    self._dispatch(self._exec.desired(selected_plan.cells if selected_plan else None))
        except (ValueError, IndexError) as exc:
            self.get_logger().error(f"Planning cycle rejected: {exc}")
            self._publish_no_route(map_msg.header.frame_id or "map", "planning_rejected", str(exc))
            if bool(self.get_parameter("execute_with_follow_path").value):
                with self._exec_lock:
                    self._dispatch(self._exec.desired(None))

    def _path_message(self, frame_id: str, plan: RoutePlan | None, grid) -> Path | None:
        if plan is None or not plan.success:
            return None
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        points = densify(centred_waypoints(grid, plan.cells), float(self.get_parameter("path_spacing_m").value))
        for idx, (wx, wy) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = wx, wy, 0.05
            if idx + 1 < len(points):
                yaw = math.atan2(points[idx + 1][1] - wy, points[idx + 1][0] - wx)
            elif idx > 0:
                yaw = math.atan2(wy - points[idx - 1][1], wx - points[idx - 1][0])
            else:
                yaw = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def _publish_no_route(self, frame_id: str, reason_code: str, explanation: str) -> None:
        empty = Path()
        empty.header.frame_id = frame_id
        empty.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(empty)
        out = String()
        out.data = json.dumps({"action": "abort", "reason_code": reason_code, "explanation": explanation,
                               "selected_route_success": False}, sort_keys=True)
        self._decision_pub.publish(out)

    def _dispatch(self, cmd: ExecCommand | None) -> None:
        """Translate an ExecutionController command into Nav2 FollowPath action-client calls."""
        if cmd is None or cmd.kind is CommandKind.NONE:
            return
        self._publish_event(cmd)
        if cmd.kind is CommandKind.CANCEL:
            handle = self._active_follow_handle
            if handle is not None:
                handle.cancel_goal_async().add_done_callback(lambda _f: None)
                self.get_logger().warning(f"Cancelling FollowPath goal ({cmd.reason_code}).")
            return
        if not self._follow_client.server_is_ready():
            # Treat as a rejected goal so the failure latch engages instead of silently retrying.
            self.get_logger().error("FollowPath action server unavailable; goal not sent.")
            self._dispatch(self._exec.on_goal_response(False))
            return
        goal = FollowPath.Goal()
        goal.path = self._path_for_cells(cmd.cells)
        goal.controller_id = str(self.get_parameter("controller_id").value)
        goal.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        goal.progress_checker_id = str(self.get_parameter("progress_checker_id").value)
        self._active_follow_handle = None
        self._follow_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _publish_event(self, cmd: ExecCommand) -> None:
        payload = {"event": cmd.kind.value, "reason_code": cmd.reason_code}
        if cmd.kind is CommandKind.SEND:
            with self._lock:
                grid = self._grid
            payload["route_xy"] = [[round(v, 3) for v in grid.cell_center_world(c)] for c in cmd.cells]
        out = String()
        out.data = json.dumps(payload)
        self._event_pub.publish(out)

    def _path_for_cells(self, cells) -> Path:
        # Rebuild from the latest map grid so the sent path matches the controller command exactly.
        with self._lock:
            grid, frame = self._grid, (self._map_msg.header.frame_id or "map")
        return self._path_message(frame, RoutePlan(tuple(cells), True, "dispatch"), grid)

    def _on_goal_response(self, future) -> None:
        with self._exec_lock:
            try:
                handle = future.result()
            except Exception as exc:  # rclpy surfaces transport failures through the future
                self.get_logger().error(f"FollowPath submission failed: {exc}")
                self._dispatch(self._exec.on_goal_response(False))
                return
            if handle.accepted:
                self._active_follow_handle = handle
                handle.get_result_async().add_done_callback(self._on_result)
            else:
                self.get_logger().warning("Nav2 rejected the FollowPath goal.")
            self._dispatch(self._exec.on_goal_response(handle.accepted))

    def _on_result(self, future) -> None:
        with self._exec_lock:
            self._active_follow_handle = None
            try:
                status = future.result().status
            except Exception as exc:
                self.get_logger().error(f"FollowPath result failed: {exc}")
                status = GoalStatus.STATUS_ABORTED
            outcome = {
                GoalStatus.STATUS_SUCCEEDED: GoalOutcome.SUCCEEDED,
                GoalStatus.STATUS_CANCELED: GoalOutcome.CANCELED,
            }.get(status, GoalOutcome.FAILED)
            self.get_logger().info(f"FollowPath terminated: status={status} -> {outcome.value}.")
            self._dispatch(self._exec.on_result(outcome))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RiskPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Ctrl-C has already shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
