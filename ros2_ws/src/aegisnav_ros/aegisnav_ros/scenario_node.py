"""ROS 2 scenario driver: publishes the mission goals, simulated confidence and ignition events.

Everything it publishes is *simulated input* (not a perception output): for scenario_name "corridor"
from the seeded `aegisnav_core.simulation.seeded_dynamic_scenario`, for other named scenarios from
their fixed `scripted_ignitions` / `scripted_confidence` schedule. Mission time starts when the first pose is received on `pose_topic`
(i.e. the robot is localized), measured on this node's clock (use_sim_time in simulation).

Interfaces (all frame_id = map frame, metres):
  publishes  goal_topic        geometry_msgs/PoseStamped    every 2 s, cell centre of the current mission goal
                                                             (waypoints in order, then base)
  publishes  /aegisnav/mission_status  std_msgs/String (JSON)  every 1 s: goal index, goals reached, phase
  subscribes decision_topic    std_msgs/String (JSON)       uses action + at_destination only
  publishes  confidence_topic  std_msgs/Float32             every 0.5 s, [0,1], simulated scalar
  publishes  ignition_topic    geometry_msgs/PointStamped   every 1 s once ignited; x,y = source position
  subscribes pose_topic        geometry_msgs/PoseWithCovarianceStamped (start trigger only)
With triage (parameter `triage`): the next goal is the remaining one with the lowest-exposure safe
route on the latest /aegisnav/hazard_forecast (aegisnav_core.timeline.triage_next_goal); a goal whose
route becomes unavailable is set aside and retried after the others.
The ignition intensity is configured on the forecast node (`ignition_intensity`).
"""
from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from aegisnav_core.models import Cell, PlannerConfig
from aegisnav_msgs.msg import HazardForecast

from aegisnav_core.batch import randomized_ignitions
from aegisnav_core.scenarios import scenario_by_name
from aegisnav_core.simulation import seeded_dynamic_scenario
from aegisnav_core.timeline import ScenarioTimeline, ScriptTimeline, next_goal, triage_next_goal

from .ros_grid import grid_from_occupancy_grid, risk_frames_from_message, world_to_cell

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class ScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_scenario")
        self.declare_parameter("seed", 0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("confidence_topic", "/aegisnav/perception_confidence")
        self.declare_parameter("ignition_topic", "/aegisnav/hazard_ignition")
        self.declare_parameter("decision_topic", "/aegisnav/decision")
        # false = sandbox: publish only the mission goal; fires and confidence come from the user.
        self.declare_parameter("scripted_events", True)
        self._scripted = bool(self.get_parameter("scripted_events").value)
        # false when confidence is estimated from sensing (hazard_sensor_node), so the two never compete.
        self.declare_parameter("publish_confidence", True)
        self.declare_parameter("scenario_name", "corridor")
        seed = int(self.get_parameter("seed").value)
        name = str(self.get_parameter("scenario_name").value)
        if name == "corridor":
            dynamic = seeded_dynamic_scenario(seed)
            self._scn = dynamic.base
            self._timeline = ScenarioTimeline(dynamic)
            schedule = (f"seed={seed}: ignition at {dynamic.ignition_time_s}s, confidence drop at "
                        f"{dynamic.confidence_drop_time_s}s to {dynamic.confidence_after}")
        else:
            self._scn = scenario_by_name(name)
            ignitions = self._scn.scripted_ignitions
            self.declare_parameter("ignition_seed", -1)  # >= 0: batch study, seeded random ignitions
            self.declare_parameter("ignition_count", 3)
            ignition_seed = int(self.get_parameter("ignition_seed").value)
            if ignition_seed >= 0:
                ignitions = randomized_ignitions(self._scn, ignition_seed, count=int(self.get_parameter("ignition_count").value))
            self._timeline = ScriptTimeline(ignitions, self._scn.scripted_confidence)
            schedule = (f"ignitions {[(t, (c.x, c.y)) for t, c, _ in ignitions]}, "
                        f"confidence {list(self._scn.scripted_confidence)}")
        self._goals = self._scn.mission_goals
        self._frame = str(self.get_parameter("map_frame").value)
        self._t0: float | None = None
        self._goal = self._goals[0]
        self._reached: list[int] = []
        self._own_goals = {*self._goals, self._scn.base}
        self._user_owns_goal = False
        self._status_pub = self.create_publisher(String, "/aegisnav/mission_status", 5)
        self.create_timer(1.0, self._publish_status)
        self._goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 5)
        self._conf_pub = self.create_publisher(Float32, str(self.get_parameter("confidence_topic").value), 5)
        self._ign_pub = self.create_publisher(PointStamped, str(self.get_parameter("ignition_topic").value), 5)
        self.create_subscription(PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value), self._on_pose, 5)
        self.create_subscription(String, str(self.get_parameter("decision_topic").value), self._on_decision, 5)
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self._on_goal, 5)
        self.declare_parameter("triage", False)
        # Planner settings for triage routes; launch passes the same values as to the planner.
        for name, default in (("nominal_speed_mps", 0.35), ("wall_clearance_cells", 1), ("wall_clearance_penalty_m", 0.3),
                              ("straight_near_obstacles", False), ("inflate_obstacles_cells", 0)):
            self.declare_parameter(name, default)
        self._triage = bool(self.get_parameter("triage").value)
        self._remaining: list[Cell] = list(self._goals)
        self._unreachable: list[Cell] = []
        self._last_reason = ""
        self._grid = None
        self._forecast: HazardForecast | None = None
        if self._triage:
            self.create_subscription(OccupancyGrid, "/aegisnav/planning_map", self._on_map, MAP_QOS)
            self.create_subscription(HazardForecast, "/aegisnav/hazard_forecast", self._on_forecast, 1)
        self.create_timer(0.5, self._publish_confidence)
        self.create_timer(1.0, self._publish_ignition)
        self.create_timer(2.0, self._publish_goal)
        if not self._scripted:
            self.get_logger().info(f"Sandbox ({name}): driving the mission; no scripted fire or confidence change. "
                                   "Set your own goal in RViz at any time.")
            return
        self.get_logger().info(f"Scenario {name} {schedule}. Waiting for first pose.")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float | None:
        return None if self._t0 is None else max(0.0, self._now_s() - self._t0)

    def _on_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        if self._t0 is None:
            self._t0 = self._now_s()
            self.get_logger().info("First pose received: mission clock started.")

    def _on_goal(self, msg: PoseStamped) -> None:
        cell = world_to_cell(self._scn.grid, msg.pose.position.x, msg.pose.position.y)
        if not self._user_owns_goal and cell not in self._own_goals:
            self._user_owns_goal = True
            self.get_logger().info("A goal was set by someone else (e.g. RViz); the scenario stops publishing goals.")

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._grid = grid_from_occupancy_grid(msg, 50)
        except ValueError as exc:
            self.get_logger().error(f"Triage map rejected: {exc}")

    def _on_forecast(self, msg: HazardForecast) -> None:
        self._forecast = msg

    def _triage_from(self, cell: Cell, candidates: list[Cell]):
        """TriageResult on the latest forecast, or None if map/forecast are missing or misaligned."""
        if self._grid is None or self._forecast is None:
            return None
        try:
            frames = risk_frames_from_message(self._forecast, self._grid, self._frame)
        except ValueError as exc:
            self.get_logger().error(f"Triage skipped: {exc}")
            return None
        cfg = PlannerConfig(
            nominal_speed_mps=float(self.get_parameter("nominal_speed_mps").value),
            wall_clearance_cells=int(self.get_parameter("wall_clearance_cells").value),
            wall_clearance_penalty_m=float(self.get_parameter("wall_clearance_penalty_m").value),
            straight_near_obstacles=bool(self.get_parameter("straight_near_obstacles").value),
            inflate_obstacles_cells=int(self.get_parameter("inflate_obstacles_cells").value),
        )
        return triage_next_goal(cell, candidates, self._scn.base, self._grid, frames, cfg,
                                time_step_s=float(self._forecast.time_step_s))

    def _on_decision(self, msg: String) -> None:
        try:
            decision = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Ignoring malformed decision JSON.")
            return
        goal_cell = decision.get("goal_cell")
        if goal_cell is not None and list(goal_cell) != [self._goal.x, self._goal.y]:
            return  # decision computed for a previous goal; acting on it would skip a waypoint
        if self._triage:
            self._triage_decision(decision)
            return
        new_goal = next_goal(self._goal, self._goals, self._scn.base,
                             at_destination=bool(decision.get("at_destination")), action=str(decision.get("action")))
        if new_goal != self._goal:
            if self._goal in self._goals:
                self._reached.append(self._goals.index(self._goal))
            label = "base" if new_goal == self._scn.base else f"waypoint {self._goals.index(new_goal) + 1}"
            self.get_logger().info(f"Goal reached; mission goal is now {label} ({new_goal.x}, {new_goal.y}).")
            self._goal = new_goal

    def _triage_decision(self, decision: dict) -> None:
        robot = decision.get("robot_cell")
        reason = str(decision.get("reason_code"))
        here = Cell(int(robot[0]), int(robot[1])) if robot else self._goal
        if decision.get("at_destination") and decision.get("action") == "continue_to_target":
            if self._goal not in self._remaining:
                return  # holding at base: mission complete
            self._reached.append(self._goals.index(self._goal))
            self._remaining.remove(self._goal)
            result = self._triage_from(here, self._remaining)
            if result is None:  # no forecast yet: fall back to the configured order
                new_goal = self._remaining[0] if self._remaining else self._scn.base
            else:
                new_goal, self._unreachable = result.goal, list(result.unreachable)
            self._switch(new_goal, "goal reached")
        elif reason == "target_route_unavailable" and reason != self._last_reason and self._goal in self._remaining:
            others = [g for g in self._remaining if g != self._goal]
            result = self._triage_from(here, others) if others else None
            if result is not None and result.goal != self._scn.base:
                self._unreachable = sorted({*result.unreachable, self._goal}, key=self._goals.index)
                self._switch(result.goal, f"no safe route to ({self._goal.x}, {self._goal.y}) now; set aside")
        self._last_reason = reason

    def _switch(self, new_goal: Cell, why: str) -> None:
        label = "base" if new_goal == self._scn.base else f"inspection point {self._goals.index(new_goal) + 1}"
        self.get_logger().info(f"Triage ({why}): mission goal is now {label} ({new_goal.x}, {new_goal.y}).")
        self._goal = new_goal

    def _publish_status(self) -> None:
        phase = "user_goal" if self._user_owns_goal else ("returning" if self._goal == self._scn.base else "outbound")
        out = String()
        out.data = json.dumps({
            "goals_total": len(self._goals), "goals_reached": sorted(set(self._reached)),
            "current_goal": [self._goal.x, self._goal.y], "phase": phase,
            "elapsed_s": round(self._elapsed() or 0.0, 1),
            "unreachable": [[c.x, c.y] for c in self._unreachable],
        })
        self._status_pub.publish(out)

    def _publish_goal(self) -> None:
        if self._t0 is None or self._user_owns_goal:
            return
        wx, wy = self._scn.grid.cell_center_world(self._goal)
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = self._frame, self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w = wx, wy, 1.0
        self._goal_pub.publish(msg)

    def _publish_confidence(self) -> None:
        if not self._scripted or not bool(self.get_parameter("publish_confidence").value):
            return
        elapsed = self._elapsed()
        if elapsed is None:
            return
        out = Float32()
        out.data = float(self._timeline.snapshot(elapsed).confidence)
        self._conf_pub.publish(out)

    def _publish_ignition(self) -> None:
        if not self._scripted:
            return
        elapsed = self._elapsed()
        if elapsed is None:
            return
        for cell, _intensity in self._timeline.snapshot(elapsed).ignited_cells:
            wx, wy = self._scn.grid.cell_center_world(cell)
            msg = PointStamped()
            msg.header.frame_id, msg.header.stamp = self._frame, self.get_clock().now().to_msg()
            msg.point.x, msg.point.y = wx, wy
            self._ign_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioNode()
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
