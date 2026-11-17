"""Batch-study recorder: what a mission actually cost, measured against the real hazard field.

Writes `output_path` (JSON, atomically, every `write_period_s` and at the end). Read by tools/analyze_batch.py.

Inputs
  /aegisnav/planning_map                    nav_msgs/OccupancyGrid     grid (transient local)
  truth_topic                               aegisnav_msgs/HazardForecast layer 0 = the real hazard field
                                            (/aegisnav/hazard_truth with sensing, else /aegisnav/hazard_forecast)
  TF map -> base_footprint                  robot position (AMCL estimate; measured ~0.1 m from Gazebo truth)
  /aegisnav/decision, /aegisnav/mission_status, /aegisnav/execution_event

The run is finished (`done: true`) when the robot is holding at base after the last goal, the policy has aborted,
the robot has held at a refuge for `refuge_hold_s`, it has made no progress for `stuck_s`, or the mission clock
passes `max_mission_s`. `outcome` is aegisnav_core.batch.classify_outcome of those facts. Exposure is a
simulated-model measure (risk x seconds at the robot's cell), not physical dose.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

from aegisnav_core.batch import RunFacts, classify_outcome
from aegisnav_core.scenarios import scenario_by_name
from aegisnav_msgs.msg import HazardForecast

from .console_state import MissionMetrics
from .ros_grid import grid_from_occupancy_grid, world_to_cell

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
REFUGE_ACTIONS = ("seek_refuge", "slow_and_reassess")


class MissionRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_mission_recorder")
        self.declare_parameter("output_path", "")
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("truth_topic", "/aegisnav/hazard_forecast")
        self.declare_parameter("max_mission_s", 900.0)
        self.declare_parameter("stuck_s", 120.0)
        self.declare_parameter("stuck_radius_m", 0.4)
        self.declare_parameter("refuge_hold_s", 30.0)
        self.declare_parameter("write_period_s", 5.0)
        self._out = str(self.get_parameter("output_path").value)
        name = str(self.get_parameter("scenario_name").value)
        if not self._out or not name:
            raise ValueError("output_path and scenario_name are required")
        self._scn = scenario_by_name(name)
        self._goals_total = len(self._scn.mission_goals)
        self._metrics = MissionMetrics()
        self._grid = None
        self._truth: list[int] = []
        self._status: dict = {}
        self._decision: dict = {}
        self._actions: dict[str, int] = {}
        self._reasons: dict[str, int] = {}
        self._sends = self._cancels = 0
        self._done_reason: str | None = None
        self._t0: float | None = None
        self._base_hold_since: float | None = None
        self._refuge_hold_since: float | None = None
        self._abort_since: float | None = None
        self._history: list[tuple[float, float, float]] = []
        self._max_still_s = 0.0
        self._timeline: list[list[float]] = []  # [wall_epoch_s, mission_s, exposure_risk_s, risk_here, x, y] every 2 s
        self._next_sample = 0.0
        self.create_subscription(OccupancyGrid, "/aegisnav/planning_map", self._on_map, MAP_QOS)
        self.create_subscription(HazardForecast, str(self.get_parameter("truth_topic").value), self._on_truth, 1)
        self.create_subscription(String, "/aegisnav/decision", self._on_decision, 10)
        self.create_subscription(String, "/aegisnav/mission_status", self._on_status, 10)
        self.create_subscription(String, "/aegisnav/execution_event", self._on_event, 10)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self, spin_thread=True)
        self.create_timer(0.5, self._sample)
        self.create_timer(float(self.get_parameter("write_period_s").value), self._write)
        self.get_logger().info(f"Recording to {self._out}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._grid = grid_from_occupancy_grid(msg, 50)
        except ValueError as exc:
            self.get_logger().error(f"Recorder map rejected: {exc}")

    def _on_truth(self, msg: HazardForecast) -> None:
        if msg.risk_layers:
            self._truth = list(msg.risk_layers[0].data)

    def _on_status(self, msg: String) -> None:
        try:
            self._status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _on_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if event.get("event") == "send":
            self._sends += 1
        elif event.get("event") == "cancel":
            self._cancels += 1

    def _on_decision(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._decision = d
        action, reason = str(d.get("action")), str(d.get("reason_code"))
        self._actions[action] = self._actions.get(action, 0) + 1
        self._reasons[reason] = self._reasons.get(reason, 0) + 1

    def _mission_s(self) -> float:
        return float(self._status.get("elapsed_s", 0.0))

    def _sample(self) -> None:
        if self._done_reason is not None or self._grid is None:
            return
        t = self._now()
        try:
            tf = self._tf.lookup_transform("map", "base_footprint", Time())
        except tf2_ros.TransformException:
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        if len(self._truth) == self._grid.size:
            cell = world_to_cell(self._grid, x, y)
            risk = self._truth[self._grid.index(cell)] / 100.0 if cell is not None else 0.0
            self._metrics.update(t, x, y, max(0.0, min(1.0, risk)))
            if t >= self._next_sample:
                self._next_sample = t + 2.0
                self._timeline.append([round(time.time(), 2), round(self._mission_s(), 1),
                                       round(self._metrics.exposure_risk_s, 2), round(risk, 3), round(x, 2), round(y, 2)])
        self._track_progress(t, x, y)
        self._check_done(t)

    def _track_progress(self, t: float, x: float, y: float) -> None:
        radius = float(self.get_parameter("stuck_radius_m").value)
        self._history.append((t, x, y))
        while self._history and t - self._history[0][0] > float(self.get_parameter("stuck_s").value):
            self._history.pop(0)
        holding = bool(self._decision.get("at_destination")) or self._decision.get("action") in ("abort",)
        span = self._history[-1][0] - self._history[0][0]
        moved = max(math.hypot(px - x, py - y) for _, px, py in self._history)
        if not holding and span >= float(self.get_parameter("stuck_s").value) - 1.0 and moved < radius:
            self._max_still_s = max(self._max_still_s, span)

    def _facts(self) -> RunFacts:
        reached = len(set(self._status.get("goals_reached", [])))
        d = self._decision
        at_base = (self._status.get("phase") == "returning" and bool(d.get("at_destination"))
                   and d.get("action") == "continue_to_target")
        return RunFacts(
            goals_total=self._goals_total, goals_reached=min(reached, self._goals_total), base_reached=at_base,
            ended_at_refuge=self._done_reason == "refuge", aborted=self._done_reason == "aborted",
            fault_latched=self._done_reason == "stuck", timed_out=self._done_reason == "timeout")

    def _check_done(self, t: float) -> None:
        d, status = self._decision, self._status
        if not d or not status:
            return
        at_dest = bool(d.get("at_destination"))
        if (status.get("phase") == "returning" and at_dest and d.get("action") == "continue_to_target"):
            self._base_hold_since = self._base_hold_since or t
            if t - self._base_hold_since >= 5.0:
                self._done_reason = "base"
        else:
            self._base_hold_since = None
        if d.get("action") == "abort":
            self._abort_since = self._abort_since or t
            if t - self._abort_since >= 10.0:
                self._done_reason = "aborted"
        else:
            self._abort_since = None
        if d.get("action") in REFUGE_ACTIONS and at_dest:
            self._refuge_hold_since = self._refuge_hold_since or t
            if t - self._refuge_hold_since >= float(self.get_parameter("refuge_hold_s").value):
                self._done_reason = "refuge"
        else:
            self._refuge_hold_since = None
        if self._max_still_s >= float(self.get_parameter("stuck_s").value) - 1.0:
            self._done_reason = "stuck"
        if self._mission_s() >= float(self.get_parameter("max_mission_s").value):
            self._done_reason = "timeout"
        if self._done_reason is not None:
            self._write()
            self.get_logger().info(f"Mission recorder finished: {self._done_reason}")

    def _write(self) -> None:
        done = self._done_reason is not None
        facts = self._facts()
        payload = {
            "done": done, "end_reason": self._done_reason,
            "outcome": classify_outcome(facts).value if done else None,
            "mission_s": round(self._mission_s(), 1), "goals_total": self._goals_total,
            "goals_reached": sorted(set(self._status.get("goals_reached", []))),
            "unreachable_now": self._status.get("unreachable", []),
            "metrics": self._metrics.as_dict(),
            "decisions": self._actions, "reason_codes": self._reasons,
            "routes_sent": self._sends, "routes_cancelled": self._cancels,
            "longest_still_s": round(self._max_still_s, 1),
            "timeline": self._timeline,
        }
        directory = os.path.dirname(self._out) or "."
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, prefix=".rec_")
        with os.fdopen(fd, "w") as f:
            json.dump(payload, f, indent=1)
        os.chmod(tmp, 0o644)  # mkstemp is 0600; the host user reads these through a bind mount
        os.replace(tmp, self._out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
