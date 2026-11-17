"""Recording director: moves the Gazebo GUI camera according to mission events (display only).

Inputs: /aegisnav/decision (action changes), /aegisnav/hazard_ignition and /clicked_point (fires).
Outputs: Gazebo GUI camera commands via the `gz` CLI on /gui/track (gz.msgs.CameraTrack) and
/gui/move_to/pose (gz.msgs.GUICamera). Needs the Gazebo GUI; do not start it headless.
"""
from __future__ import annotations

import json
import math
import subprocess

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String

from .director import Director, Shot

OVERVIEW = (0.0, -19.0, 11.0, 0.62, 1.5708)   # x, y, z, pitch, yaw (world frame)
TACTICAL = (0.0, -0.5, 23.0, 1.5707, 1.5708)


def _quat(pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cp, sp, cy, sy = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    return (-sp * sy, sp * cy, cp * sy, cp * cy)  # roll = 0


class DirectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_director")
        self._director = Director()
        self._last_action = None
        self._current: Shot | None = None
        self._repeats_left = 0
        self._t0 = None
        self.create_subscription(String, "/aegisnav/decision", self._on_decision, 5)
        self.create_subscription(PointStamped, "/aegisnav/hazard_ignition", self._on_fire, 5)
        self.create_subscription(PointStamped, "/clicked_point", self._on_fire, 5)
        self.create_subscription(PointStamped, "/aegisnav/hazard_discovered", self._on_fire, 5)  # sensing mode
        self._seen_fires: set = set()
        self.create_timer(0.5, self._tick)

    def _t(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        return now - self._t0

    def _on_decision(self, msg: String) -> None:
        try:
            action = json.loads(msg.data).get("action")
        except json.JSONDecodeError:
            return
        if self._last_action is not None and action != self._last_action:
            self._director.on_decision_change(self._t())
        self._last_action = action

    def _on_fire(self, msg: PointStamped) -> None:
        key = (round(msg.point.x, 1), round(msg.point.y, 1))
        if key not in self._seen_fires:
            self._seen_fires.add(key)
            self._director.on_fire(self._t(), msg.point.x, msg.point.y)

    def _gz(self, args: list[str]) -> None:
        try:
            subprocess.run(["gz", *args], check=True, capture_output=True, timeout=5)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().warning(f"Camera command failed: {exc}", throttle_duration_sec=10.0)

    def _move_to(self, x, y, z, pitch, yaw) -> None:
        qx, qy, qz, qw = _quat(pitch, yaw)
        self._gz(["topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p", "track_mode: NONE"])
        self._gz(["service", "-s", "/gui/move_to/pose", "--reqtype", "gz.msgs.GUICamera", "--reptype", "gz.msgs.Boolean",
                  "--timeout", "2000", "--req",
                  f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}"])

    def _tick(self) -> None:
        shot = self._director.shot(self._t())
        if shot != self._current:
            self._current = shot
            # `gz topic -p` is a one-shot publish from a fresh process and can be lost before discovery
            # completes (observed: the camera stayed on a fire shot); repeat each shot on the next ticks.
            self._repeats_left = 3
            self.get_logger().info(f"Camera: {shot.kind}")
        if self._repeats_left == 0:
            return
        self._repeats_left -= 1
        if shot.kind == "overview":
            self._move_to(*OVERVIEW)
        elif shot.kind == "tactical":
            self._move_to(*TACTICAL)
        elif shot.kind == "fire":
            fx, fy = shot.target_xy
            # 4 m south, 5 m up, pitched 0.9 rad: the optical axis meets the floor at the fire.
            self._move_to(fx, fy - 4.0, 5.0, 0.9, 1.5708)
        else:  # chase: follow the robot from behind and above
            self._gz(["topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p",
                      "track_mode: FOLLOW_LOOK_AT, follow_target: {name: 'aegis_bot'}, track_target: {name: 'aegis_bot'}, "
                      "follow_offset: {x: -4.0, y: 0.0, z: 3.0}, follow_pgain: 0.05, track_pgain: 0.05"])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectorNode()
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
