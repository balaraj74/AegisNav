"""Operator console: serves a self-contained web page and streams live mission state to it.

Display and operator input only — the planner never reads anything from here except the inputs
the operator sends, which go out on the same topics RViz/CLI use:
  POST /api/fire {x,y}        -> /clicked_point (simulated ignition)
  POST /api/goal {x,y}        -> /goal_pose
  POST /api/confidence {value}-> /aegisnav/perception_confidence (simulated scalar)
  GET  /api/static            -> planning grid + mission geometry (from the core scenario)
  GET  /api/stream            -> server-sent events, ~4 Hz state snapshots
Parameters: port (8080), bind_address ("0.0.0.0" inside the container; publish to host localhost only).
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, String

from aegisnav_core.scenarios import scenario_by_name
from aegisnav_msgs.msg import HazardForecast

from .console_state import MissionMetrics, parse_confidence, parse_point
from .event_log import EventLog
from .ros_grid import grid_from_occupancy_grid, world_to_cell

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class ConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_console")
        self.declare_parameter("port", 8080)
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("map_topic", "/aegisnav/planning_map")
        self.declare_parameter("predicted_step", 5)  # forecast layer index shown as "predicted"
        self._lock = threading.Lock()
        self._grid = None
        self._frame = "map"
        self._risk_now: list[int] = []
        self._risk_pred: list[int] = []
        self._pred_seconds = 0.0
        self._decision: dict = {}
        self._status: dict = {}
        self._route: list = []
        self._ghost: list = []
        self._robot: dict | None = None
        self._events = EventLog(max_lines=9)
        self._metrics = MissionMetrics()
        self._t0: float | None = None
        self._last_key = None
        name = str(self.get_parameter("scenario_name").value)
        self._scn = scenario_by_name(name) if name else None
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, MAP_QOS)
        self.create_subscription(HazardForecast, "/aegisnav/hazard_forecast", self._on_forecast, 1)
        # Sensing mode only: the real field the robot cannot see, for the viewer's comparison.
        self.create_subscription(HazardForecast, "/aegisnav/hazard_truth", self._on_truth, 1)
        self._risk_truth: list[int] = []
        self.create_subscription(String, "/aegisnav/decision", self._on_decision, 5)
        self.create_subscription(String, "/aegisnav/mission_status", self._on_status, 5)
        self.create_subscription(String, "/aegisnav/execution_event", self._on_event, 10)
        self.create_subscription(Path, "/aegisnav/planned_path", self._on_path, 5)
        self.create_subscription(PointStamped, "/aegisnav/hazard_ignition", self._on_fire, 5)
        self.create_subscription(PointStamped, "/clicked_point", self._on_fire, 5)
        # Sensing mode: what the robot's simulated onboard sensor found (new hazard regions, victims).
        self.create_subscription(PointStamped, "/aegisnav/hazard_discovered", self._on_discovered, 5)
        self.create_subscription(String, "/aegisnav/victim_detections", self._on_victims, 5)
        self._victims_found: list = []
        self._fire_pub = self.create_publisher(PointStamped, "/clicked_point", 5)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 5)
        self._conf_pub = self.create_publisher(Float32, "/aegisnav/perception_confidence", 5)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self, spin_thread=True)
        self.create_timer(0.2, self._poll_pose)
        self._page = os.path.join(get_package_share_directory("aegisnav_ros"), "console", "index.html")
        self._server = ThreadingHTTPServer((str(self.get_parameter("bind_address").value),
                                            int(self.get_parameter("port").value)), self._handler_class())
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info(f"Operator console on port {self.get_parameter('port').value}.")

    # --- ROS inputs -------------------------------------------------------------------
    def _now(self) -> float:
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            grid = grid_from_occupancy_grid(msg, 50)
        except ValueError as exc:
            self.get_logger().error(f"Console map rejected: {exc}")
            return
        with self._lock:
            self._grid, self._frame = grid, msg.header.frame_id or "map"

    def _on_forecast(self, msg: HazardForecast) -> None:
        if not msg.risk_layers:
            return
        step = min(int(self.get_parameter("predicted_step").value), len(msg.risk_layers) - 1)
        with self._lock:
            self._risk_now = list(msg.risk_layers[0].data)
            self._risk_pred = list(msg.risk_layers[step].data)
            self._pred_seconds = step * float(msg.time_step_s)

    def _on_truth(self, msg: HazardForecast) -> None:
        if msg.risk_layers:
            with self._lock:
                self._risk_truth = list(msg.risk_layers[0].data)

    def _on_decision(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._decision = d
            key = (d.get("action"), d.get("reason_code"))
            if key != self._last_key:
                self._events.add(self._now(), f"Decision {str(key[0]).upper()} ({key[1]})")
                self._last_key = key

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            before = set(self._status.get("goals_reached", []))
            for i in sorted(set(status.get("goals_reached", [])) - before):
                self._events.add(self._now(), f"Victim {i + 1} reached")
            self._status = status

    def _on_event(self, msg: String) -> None:
        try:
            e = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            if e.get("event") == "cancel":
                self._ghost = self._route
                self._events.add(self._now(), f"Route cancelled ({e.get('reason_code')})")
            elif e.get("event") == "send" and e.get("reason_code") == "replacement_dispatch":
                self._events.add(self._now(), "New route dispatched")

    def _on_path(self, msg: Path) -> None:
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in msg.poses[::2]]
        with self._lock:
            self._route = pts

    def _on_fire(self, msg: PointStamped) -> None:
        x, y = msg.point.x, msg.point.y
        with self._lock:
            self._events.add(self._now(), f"FIRE at ({x:.1f}, {y:.1f})", key=f"{x:.1f}:{y:.1f}")

    def _on_discovered(self, msg: PointStamped) -> None:
        x, y = msg.point.x, msg.point.y
        with self._lock:
            self._events.add(self._now(), f"FIRE DETECTED by robot sensor at ({x:.1f}, {y:.1f})", key=f"det{x:.1f}:{y:.1f}")

    def _on_victims(self, msg: String) -> None:
        try:
            found = json.loads(msg.data).get("found", [])
        except (json.JSONDecodeError, AttributeError):
            return
        with self._lock:
            for cell in found:
                if cell not in self._victims_found:
                    self._victims_found.append(cell)
                    self._events.add(self._now(), f"VICTIM FOUND by robot sensor at cell ({cell[0]}, {cell[1]})")

    def _poll_pose(self) -> None:
        try:
            tf = self._tf.lookup_transform("map", "base_footprint", Time())
        except tf2_ros.TransformException:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = tf.transform.translation.x, tf.transform.translation.y
        with self._lock:
            self._robot = {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)}
            # What the robot experienced is measured against the real field: the hidden truth when sensing is
            # on (the forecast layer is then only the robot's belief), else the forecast's current layer.
            field = self._risk_truth if self._risk_truth else self._risk_now
            if self._grid is not None and len(field) == self._grid.size:
                cell = world_to_cell(self._grid, x, y)
                risk = field[self._grid.index(cell)] / 100.0 if cell is not None else 0.0
                self._metrics.update(self._now(), x, y, max(0.0, min(1.0, risk)))

    # --- HTTP --------------------------------------------------------------------------
    def static_payload(self) -> dict:
        with self._lock:
            g = self._grid
            if g is None:
                return {}
            out = {"width": g.width, "height": g.height, "resolution": g.resolution_m,
                   "origin": [g.origin_x_m, g.origin_y_m], "blocked": sorted(g.blocked)}
        scn = self._scn
        if scn is not None:
            out.update({"scenario": scn.scenario_id, "base": [scn.base.x, scn.base.y],
                        "refuges": [[c.x, c.y] for c in scn.refuges], "victims": [[c.x, c.y] for c in scn.victims],
                        "goals": [[c.x, c.y] for c in scn.mission_goals]})
        return out

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "t": round(self._now(), 1), "robot": self._robot, "route": self._route, "ghost": self._ghost,
                "risk_now": self._risk_now, "risk_pred": self._risk_pred, "pred_seconds": self._pred_seconds,
                "risk_truth": self._risk_truth,
                "decision": {k: v for k, v in self._decision.items() if k != "model_notice"},
                "status": self._status, "metrics": self._metrics.as_dict(), "events": self._events.lines(),
                "victims_found": self._victims_found,
            }

    def publish_fire(self, x: float, y: float) -> None:
        msg = PointStamped()
        msg.header.frame_id, msg.header.stamp = self._frame, self.get_clock().now().to_msg()
        msg.point.x, msg.point.y = x, y
        self._fire_pub.publish(msg)

    def publish_goal(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = self._frame, self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w = x, y, 1.0
        self._goal_pub.publish(msg)

    def publish_confidence(self, value: float) -> None:
        self._conf_pub.publish(Float32(data=value))

    def _handler_class(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args) -> None:  # keep the ROS log clean
                return

            def _send(self, code: int, body: bytes, ctype: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:
                if self.path in ("/", "/index.html"):
                    with open(node._page, "rb") as f:
                        self._send(200, f.read(), "text/html; charset=utf-8")
                elif self.path == "/api/static":
                    self._send(200, json.dumps(node.static_payload()).encode(), "application/json")
                elif self.path == "/api/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    try:
                        while rclpy.ok():
                            self.wfile.write(b"data: " + json.dumps(node.snapshot()).encode() + b"\n\n")
                            self.wfile.flush()
                            time.sleep(0.25)
                    except (BrokenPipeError, ConnectionResetError):
                        return
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self) -> None:
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 4096:
                        raise ValueError("bad body size")
                    payload = json.loads(self.rfile.read(length))
                    if self.path == "/api/fire":
                        node.publish_fire(*parse_point(payload))
                    elif self.path == "/api/goal":
                        node.publish_goal(*parse_point(payload))
                    elif self.path == "/api/confidence":
                        node.publish_confidence(parse_confidence(payload))
                    else:
                        self._send(404, b"not found", "text/plain")
                        return
                    self._send(200, b'{"ok": true}', "application/json")
                except (ValueError, KeyError, json.JSONDecodeError) as exc:
                    self._send(400, json.dumps({"error": str(exc)}).encode(), "application/json")

        return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._server.shutdown()
        node.destroy_node()
        if rclpy.ok():  # Ctrl-C has already shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
