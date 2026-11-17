"""Visualization only: draws AegisNav data in Gazebo and RViz. Never an input to planning.

  subscribes /aegisnav/hazard_forecast (layer 0)  -> Gazebo floor tiles via the bridged
             /world/<world>/create and /remove services (ros_gz_interfaces SpawnEntity/DeleteEntity)
  subscribes /aegisnav/planning_map                -> cell geometry for the tiles
  subscribes /aegisnav/decision, /goal_pose,       -> RViz /aegisnav/mission_markers
             /aegisnav/execution_event,          (base, refuges, goal, decision + explanation,
             /aegisnav/hazard_ignition, /clicked_point   event log, grey ghost of the last cancelled route)
If the Gazebo services are unavailable the tiles are skipped and a warning is logged; RViz
markers do not depend on Gazebo.
"""
from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from aegisnav_core.models import Cell
from aegisnav_msgs.msg import HazardForecast

from .event_log import EventLog, wrap
from .hazard_tiles import band_model_name, band_model_sdf, cells_by_band, desired_bands, effect_model_sdf
from aegisnav_core.scenarios import scenario_by_name

from .ros_grid import apply_scenario_parameters, grid_from_occupancy_grid, world_to_cell

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class VisualizationNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_visualization")
        self.declare_parameter("world_name", "aegisnav_rescue_world")
        self.declare_parameter("map_topic", "/aegisnav/planning_map")
        self.declare_parameter("forecast_topic", "/aegisnav/hazard_forecast")
        self.declare_parameter("decision_topic", "/aegisnav/decision")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("base_cell_xy", [2, 2])
        self.declare_parameter("refuge_cells_xy", [2, 12, 21, 2, 6, 12])
        self.declare_parameter("gazebo_tiles", True)
        self.declare_parameter("tile_period_s", 2.0)  # seconds between Gazebo tile refreshes
        name = str(self.get_parameter("scenario_name").value)
        self._victims: list[Cell] = []
        # Source markers: cell -> kind for every hazard source to draw; spawned once each.
        self._effects_wanted: dict[Cell, str] = {}
        self._effects_done: set[Cell] = set()
        if name:
            scn = scenario_by_name(name)
            self._victims = list(scn.victims)
            for src in scn.sources:
                self._effects_wanted[src.cell] = "gas" if src.label == "gas" else "fire"
            apply_scenario_parameters(self, name, {
                "base_cell_xy": [scn.base.x, scn.base.y],
                "refuge_cells_xy": [v for c in scn.refuges for v in (c.x, c.y)],
            })
        world = str(self.get_parameter("world_name").value)
        self._tile_period = float(self.get_parameter("tile_period_s").value)
        if not self._tile_period > 0:
            raise ValueError("tile_period_s must be positive")
        self._next_tiles_s = 0.0
        self._grid = None
        self._frame = "map"
        self._latest_risk = None
        self._drawn: dict[int, tuple[frozenset, str]] = {}  # band -> (cells, model name)
        self._orphans: list[str] = []  # tiles whose spawn/remove reply was lost; removed later
        self._pending: dict = {}  # future -> (deadline_s, client, on_timeout)
        self._generation = 0
        self._decision: dict = {}
        self._last_decision_key = None
        self._last_confidence = None
        self._events = EventLog(max_lines=7)
        self._t0: float | None = None
        self._active_route: list = []
        self._ghost_route: list = []
        self._goal: PoseStamped | None = None
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, MAP_QOS)
        self.create_subscription(HazardForecast, str(self.get_parameter("forecast_topic").value), self._on_forecast, 1)
        self.create_subscription(String, str(self.get_parameter("decision_topic").value), self._on_decision, 5)
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self._on_goal, 5)
        self.create_subscription(String, "/aegisnav/execution_event", self._on_execution_event, 10)
        self.create_subscription(PointStamped, "/aegisnav/hazard_ignition", self._on_fire, 5)
        self.create_subscription(PointStamped, "/clicked_point", self._on_fire, 5)
        self._markers = self.create_publisher(MarkerArray, "/aegisnav/mission_markers", 1)
        self._spawn = self.create_client(SpawnEntity, f"/world/{world}/create")
        self._remove = self.create_client(DeleteEntity, f"/world/{world}/remove")
        self.create_timer(0.5, self._tick)

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._grid = grid_from_occupancy_grid(msg, 50)
        except ValueError as exc:
            self.get_logger().error(f"Visualization map rejected: {exc}")
            return
        self._frame = msg.header.frame_id or "map"

    def _on_forecast(self, msg: HazardForecast) -> None:
        if msg.risk_layers:
            self._latest_risk = [v / 100.0 for v in msg.risk_layers[0].data]

    def _elapsed(self) -> float:
        now = self._now_s()
        if self._t0 is None:
            self._t0 = now
        return now - self._t0

    def _on_decision(self, msg: String) -> None:
        try:
            self._decision = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Ignoring malformed decision JSON.")
            return
        d = self._decision
        key = (d.get("action"), d.get("reason_code"))
        if key != self._last_decision_key:
            self._events.add(self._elapsed(), f"decision {str(key[0]).upper()} ({key[1]})")
            self._last_decision_key = key
        conf = d.get("perception_confidence")
        if conf is not None and (self._last_confidence is None or abs(conf - self._last_confidence) > 0.05):
            if self._last_confidence is not None:
                self._events.add(self._elapsed(), f"perception confidence -> {conf:.2f} (simulated)")
            self._last_confidence = conf

    def _on_goal(self, msg: PoseStamped) -> None:
        if self._goal is None or (abs(self._goal.pose.position.x - msg.pose.position.x) > 1e-3
                                  or abs(self._goal.pose.position.y - msg.pose.position.y) > 1e-3):
            self._events.add(self._elapsed(), f"goal set ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})")
        self._goal = msg

    def _on_fire(self, msg: PointStamped) -> None:
        x, y = msg.point.x, msg.point.y
        self._events.add(self._elapsed(), f"FIRE started at ({x:.1f}, {y:.1f})", key=f"fire:{x:.1f}:{y:.1f}")
        if self._grid is not None:
            cell = world_to_cell(self._grid, x, y)
            if cell is not None and not self._grid.is_blocked(cell) and len(self._effects_wanted) < 12:
                self._effects_wanted.setdefault(cell, "fire")

    def _on_execution_event(self, msg: String) -> None:
        try:
            e = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if e.get("event") == "send":
            self._active_route = e.get("route_xy", [])
            label = "new route sent" if e.get("reason_code") == "replacement_dispatch" else "route sent"
            self._events.add(self._elapsed(), f"{label} ({len(self._active_route)} cells)")
        elif e.get("event") == "cancel":
            self._ghost_route = self._active_route
            self._events.add(self._elapsed(), f"route CANCELLED ({e.get('reason_code')})")

    def _tick(self) -> None:
        if self._grid is None:
            return
        self._elapsed()
        self._publish_mission_markers()
        if bool(self.get_parameter("gazebo_tiles").value) and self._latest_risk is not None:
            self._update_tiles()

    # --- Gazebo tiles -------------------------------------------------------------------
    def _update_tiles(self) -> None:
        self._expire_calls()
        if self._pending or self._now_s() < self._next_tiles_s:
            return  # previous batch not answered yet, or not due; keeps calls bounded
        if not (self._spawn.service_is_ready() and self._remove.service_is_ready()):
            self.get_logger().warning("Gazebo create/remove services not available; hazard tiles skipped.",
                                      throttle_duration_sec=10.0)
            return
        if len(self._latest_risk) != self._grid.size:
            self.get_logger().error("Forecast layer size does not match planning map; tiles skipped.")
            return
        self._next_tiles_s = self._now_s() + self._tile_period
        self._update_effects()
        for name in list(self._orphans):
            self._orphans.remove(name)
            self._remove_tile(name)
        for band, cells in cells_by_band(desired_bands(self._grid.width, self._latest_risk, self._grid.blocked)).items():
            drawn_cells, old_name = self._drawn.get(band, (frozenset(), None))
            if cells == drawn_cells:
                continue
            if not cells:
                self._drawn.pop(band, None)
                self._remove_tile(old_name)
                continue
            self._generation += 1
            name = band_model_name(band, self._generation)
            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.sdf = band_model_sdf(
                name, band, [self._grid.cell_center_world(c) for c in sorted(cells, key=lambda c: (c.y, c.x))],
                self._grid.resolution_m * 0.92)

            def spawned(ok, band=band, cells=cells, name=name, old=old_name):
                if ok:  # replace, then remove the old model: no frame without tiles
                    self._drawn[band] = (cells, name)
                    if old:
                        self._remove_tile(old)
            self._call(self._spawn, req, on_done=spawned, on_timeout=lambda n=name: self._orphans.append(n))
        self.get_logger().info(
            "hazard tiles drawn per band: " + ", ".join(f"{b}:{len(c)}" for b, (c, _) in sorted(self._drawn.items())),
            throttle_duration_sec=5.0)

    def _update_effects(self) -> None:
        """Spawn one source-marker model (flame/smoke or gas haze) per hazard source (display only)."""
        for cell, kind in list(self._effects_wanted.items()):
            if cell in self._effects_done:
                continue
            self._effects_done.add(cell)  # one attempt per cell; a failure is logged, not retried in a loop
            self._generation += 1
            name = f"aegisnav_fx_{kind}_{cell.x}_{cell.y}_g{self._generation}"
            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.sdf = effect_model_sdf(name, kind)
            x, y = self._grid.cell_center_world(cell)
            req.entity_factory.pose.position.x, req.entity_factory.pose.position.y = x, y

            def done(ok, cell=cell):
                if not ok:
                    self.get_logger().warning(f"Could not spawn effect at cell ({cell.x}, {cell.y}).")
            self._call(self._spawn, req, on_done=done, on_timeout=lambda n=name: self._orphans.append(n))

    def _remove_tile(self, name: str) -> None:
        req = DeleteEntity.Request()
        req.entity.name, req.entity.type = name, Entity.MODEL
        # A failed removal means the model is already absent; a lost reply leaves it uncertain.
        self._call(self._remove, req, on_done=lambda ok: None, on_timeout=lambda n=name: self._orphans.append(n))

    def _call(self, client, request, *, on_done, on_timeout, timeout_s: float = 3.0) -> None:
        future = client.call_async(request)
        self._pending[future] = (self._now_s() + timeout_s, client, on_timeout)

        def done(fut):
            if self._pending.pop(fut, None) is None:
                return  # already expired
            try:
                ok = bool(fut.result().success)
            except Exception as exc:  # rclpy surfaces transport errors through the future
                self.get_logger().error(f"Gazebo tile service call failed: {exc}")
                ok = False
            on_done(ok)

        future.add_done_callback(done)

    def _expire_calls(self) -> None:
        now = self._now_s()
        for future, (deadline, client, on_timeout) in list(self._pending.items()):
            if now >= deadline:
                del self._pending[future]
                client.remove_pending_request(future)
                self.get_logger().warning("Gazebo tile service reply lost; will reconcile.", throttle_duration_sec=10.0)
                on_timeout()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- RViz mission markers -----------------------------------------------------------
    def _cells(self, name: str) -> list[Cell]:
        v = list(self.get_parameter(name).value)
        return [Cell(int(v[i]), int(v[i + 1])) for i in range(0, len(v) - 1, 2)]

    def _marker(self, mid: int, mtype: int, x: float, y: float, rgba, scale) -> Marker:
        m = Marker()
        m.header.frame_id, m.header.stamp = self._frame, self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "aegisnav_mission", mid, mtype, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.orientation.w = x, y, 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _publish_mission_markers(self) -> None:
        g, out = self._grid, MarkerArray()
        r = g.resolution_m
        for cell in self._cells("base_cell_xy"):
            out.markers.append(self._marker(1, Marker.CYLINDER, *g.cell_center_world(cell), (0.15, 0.45, 1.0, 0.9), (r, r, 0.04)))
        for i, cell in enumerate(self._cells("refuge_cells_xy")):
            out.markers.append(self._marker(10 + i, Marker.CYLINDER, *g.cell_center_world(cell), (0.1, 0.85, 0.35, 0.9), (r, r, 0.04)))
        for i, cell in enumerate(self._victims):
            out.markers.append(self._marker(20 + i, Marker.CYLINDER, *g.cell_center_world(cell), (1.0, 0.25, 0.6, 0.95), (r, r, 0.6)))
        if self._goal is not None:
            p = self._goal.pose.position
            out.markers.append(self._marker(2, Marker.SPHERE, p.x, p.y, (1.0, 0.85, 0.1, 0.95), (0.22, 0.22, 0.22)))
        cx = g.origin_x_m + g.width * r / 2
        if self._decision:
            d = self._decision
            text = self._marker(3, Marker.TEXT_VIEW_FACING, cx, g.origin_y_m + g.height * r + 0.9,
                                (1.0, 1.0, 1.0, 1.0), (0.0, 0.0, 0.26))
            text.text = (f"{d.get('action', '?').upper()}  |  {d.get('reason_code', '?')}  |  "
                         f"confidence {d.get('perception_confidence', 0):.2f} (simulated)\n"
                         f"{wrap(str(d.get('explanation', '')), 78)}")
            out.markers.append(text)
        if self._events.lines():
            log = self._marker(4, Marker.TEXT_VIEW_FACING, cx, g.origin_y_m - 1.0, (0.85, 0.95, 1.0, 1.0), (0.0, 0.0, 0.2))
            log.text = "\n".join(self._events.lines())
            out.markers.append(log)
        ghost = self._marker(5, Marker.LINE_STRIP, 0.0, 0.0, (0.6, 0.6, 0.65, 0.8), (0.05, 0.0, 0.0))
        ghost.points = [Point(x=float(x), y=float(y), z=0.03) for x, y in self._ghost_route]
        if len(ghost.points) < 2:
            ghost.action = Marker.DELETE
        out.markers.append(ghost)
        self._markers.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizationNode()
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
