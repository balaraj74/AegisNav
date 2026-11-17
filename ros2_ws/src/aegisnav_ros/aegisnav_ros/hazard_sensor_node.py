"""Simulated onboard hazard sensor and victim detector.

Reads the *true* simulated hazard field (layer 0 of `truth_topic`, published by a forecast node in
source_mode "truth") and reports it only where the robot can see: cells within `range_m` and in line of
sight on the planning grid (aegisnav_core.sensing). Simulation assumption, not a thermal-camera model.

Interfaces (frame = map frame of the planning map):
  subscribes map_topic                  nav_msgs/OccupancyGrid (transient local)
  subscribes truth_topic                aegisnav_msgs/HazardForecast; only layer 0 (current truth) is used
  TF         map_frame -> base_frame    robot pose
  publishes  /aegisnav/hazard_observation  nav_msgs/OccupancyGrid every period_s: 0..100 = observed risk
                                           (normalized x100), -1 = not seen this scan
  publishes  /aegisnav/victim_detections   std_msgs/String (JSON) every period_s:
                                           {"found": [[x, y], ...]} victim cells seen so far (scenario victims)
  subscribes pose_topic                    geometry_msgs/PoseWithCovarianceStamped (AMCL), covariance only
  publishes  /aegisnav/perception_confidence  std_msgs/Float32 every period_s, only if estimate_confidence:
                                           aegisnav_core.sensing.estimate_confidence (heuristic, not calibrated)
Dense smoke: cells whose true risk >= smoke_threshold are seen but hide what is behind them.
Fails closed: without a fresh pose (TF) or an aligned truth layer, nothing is published.
"""
from __future__ import annotations

import json
import random

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, String

from aegisnav_core.scenarios import scenario_by_name
from aegisnav_core.sensing import detected, estimate_confidence, observe, visible_cells
from aegisnav_msgs.msg import HazardForecast

from .ros_grid import grid_from_occupancy_grid, observation_data, risk_frames_from_message, snap_to_free_cell

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class HazardSensorNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_hazard_sensor")
        self.declare_parameter("map_topic", "/aegisnav/planning_map")
        self.declare_parameter("truth_topic", "/aegisnav/hazard_truth")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("range_m", 4.0)  # simulated sensing range, metres
        self.declare_parameter("noise_sigma", 0.03)  # normalized-risk noise, standard deviation
        self.declare_parameter("seed", 0)
        self.declare_parameter("period_s", 0.5)
        self.declare_parameter("max_pose_age_s", 1.0)
        self.declare_parameter("snap_tolerance_m", 0.3)
        self.declare_parameter("smoke_threshold", 0.6)  # true risk at which a cell blocks the view
        self.declare_parameter("estimate_confidence", False)
        self.declare_parameter("pose_topic", "/amcl_pose")
        range_m, noise = float(self.get_parameter("range_m").value), float(self.get_parameter("noise_sigma").value)
        if not 0.0 < range_m <= 50.0 or not 0.0 <= noise <= 0.5:
            raise ValueError("range_m must be in (0, 50] and noise_sigma in [0, 0.5]")
        name = str(self.get_parameter("scenario_name").value)
        self._victims = scenario_by_name(name).victims if name else ()
        self._rng = random.Random(int(self.get_parameter("seed").value))
        self._grid = None
        self._map_msg: OccupancyGrid | None = None
        self._truth: tuple[float, ...] | None = None
        self._found: list = []
        self._loc_std_m: float | None = None
        self.create_subscription(PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value),
                                 self._on_pose, 5)
        self._conf_pub = self.create_publisher(Float32, "/aegisnav/perception_confidence", 5)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, MAP_QOS)
        self.create_subscription(HazardForecast, str(self.get_parameter("truth_topic").value), self._on_truth, 1)
        self._obs_pub = self.create_publisher(OccupancyGrid, "/aegisnav/hazard_observation", 5)
        self._victim_pub = self.create_publisher(String, "/aegisnav/victim_detections", 5)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self, spin_thread=True)
        self.create_timer(max(0.1, float(self.get_parameter("period_s").value)), self._scan)
        self.get_logger().info(f"Simulated hazard sensor: range {range_m} m, noise sigma {noise}.")

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._grid = grid_from_occupancy_grid(msg, 50)
        except ValueError as exc:
            self.get_logger().error(f"Sensor map rejected: {exc}")
            return
        self._map_msg = msg

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        cov = msg.pose.covariance
        var = max(float(cov[0]), float(cov[7]))  # x and y position variance (m^2)
        self._loc_std_m = math.sqrt(var) if math.isfinite(var) and var >= 0 else None

    def _on_truth(self, msg: HazardForecast) -> None:
        if self._grid is None or self._map_msg is None or not msg.risk_layers:
            return
        first = HazardForecast(header=msg.header, time_step_s=msg.time_step_s,
                               risk_layers=msg.risk_layers[:1], uncertainty_layers=msg.uncertainty_layers[:1])
        try:
            self._truth = risk_frames_from_message(first, self._grid, self._map_msg.header.frame_id or "map")[0].risk
        except ValueError as exc:
            self._truth = None
            self.get_logger().error(f"Truth hazard layer rejected: {exc}")

    def _scan(self) -> None:
        if self._grid is None or self._map_msg is None or self._truth is None:
            return
        try:
            tf = self._tf.lookup_transform(str(self.get_parameter("map_frame").value),
                                           str(self.get_parameter("base_frame").value), Time())
        except tf2_ros.TransformException:
            return
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > float(self.get_parameter("max_pose_age_s").value):
            self.get_logger().warning(f"Pose is {age:.1f} s old; not sensing.", throttle_duration_sec=5.0)
            return
        cell = snap_to_free_cell(self._grid, tf.transform.translation.x, tf.transform.translation.y,
                                 float(self.get_parameter("snap_tolerance_m").value))
        if cell is None:
            return
        range_m = float(self.get_parameter("range_m").value)
        smoke = frozenset(i for i, r in enumerate(self._truth) if r >= float(self.get_parameter("smoke_threshold").value))
        visible = visible_cells(self._grid, cell, range_m, obscured=smoke)
        readings = observe(self._truth, (i for i in visible if i not in self._grid.blocked),
                           noise_sigma=float(self.get_parameter("noise_sigma").value), rng=self._rng)
        out = OccupancyGrid()
        out.header.frame_id = self._map_msg.header.frame_id or "map"
        out.header.stamp = tf.header.stamp
        out.info = self._map_msg.info
        out.data = observation_data(readings, self._grid.size)
        self._obs_pub.publish(out)
        for victim in detected(visible, self._grid, self._victims):
            if victim not in self._found:
                self._found.append(victim)
                self.get_logger().warning(f"Victim detected by onboard sensor at cell ({victim.x}, {victim.y}).")
        self._victim_pub.publish(String(data=json.dumps({"found": [[v.x, v.y] for v in self._found]})))
        if bool(self.get_parameter("estimate_confidence").value):
            if self._loc_std_m is None:
                return  # no localization estimate yet: publish nothing (the planner keeps waiting)
            clear = visible_cells(self._grid, cell, range_m)
            self._conf_pub.publish(Float32(data=float(estimate_confidence(len(visible), len(clear), self._loc_std_m))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HazardSensorNode()
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
