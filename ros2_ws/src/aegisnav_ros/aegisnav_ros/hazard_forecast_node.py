"""ROS 2 adapter: publish repeatable risk/uncertainty forecast layers from /map.

source_mode "truth" (default): forecast from the configured sources and ignitions. These are
simulator-only configuration inputs and must not be misrepresented as real fire/gas measurements.
source_mode "belief": forecast only from what the simulated onboard sensor has reported on
`observation_topic` (aegisnav_core.sensing.HazardBelief); configured sources and ignitions are ignored,
and unseen cells carry high uncertainty instead of being assumed safe.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from aegisnav_msgs.msg import HazardForecast

from aegisnav_core.forecast import HazardForecaster
from aegisnav_core.models import Cell, ForecastConfig, HazardSource
from aegisnav_core.scenarios import scenario_by_name
from aegisnav_core.sensing import HazardBelief
from .ros_grid import apply_scenario_parameters, grid_from_occupancy_grid, observation_from_message, world_to_cell


# map_server publishes the static map once, latched; late subscribers need transient-local.
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class HazardForecastNode(Node):
    def __init__(self) -> None:
        super().__init__("aegisnav_hazard_forecast")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("forecast_topic", "/aegisnav/hazard_forecast")
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("source_cells_xy", [11, 3])
        self.declare_parameter("source_intensities", [0.94])
        self.declare_parameter("forecast_steps", 120)
        self.declare_parameter("time_step_s", 1.0)
        self.declare_parameter("spread_rate", 0.24)
        self.declare_parameter("decay_rate", 0.008)
        self.declare_parameter("uncertainty_growth", 0.018)
        self.declare_parameter("update_period_s", 1.0)
        self.declare_parameter("occupied_threshold", 50)
        # Ignition events: PointStamped (map frame, metres) on this topic add a persistent
        # simulated source of the given normalized intensity [0,1]. Simulated input only.
        self.declare_parameter("ignition_topic", "/aegisnav/hazard_ignition")
        self.declare_parameter("ignition_intensity", 0.95)
        self.declare_parameter("source_mode", "truth")  # "truth" | "belief"
        self.declare_parameter("observation_topic", "/aegisnav/hazard_observation")
        self.declare_parameter("belief_prior_uncertainty", 0.6)  # unseen cells, normalized
        self.declare_parameter("belief_source_threshold", 0.5)  # observed risk that counts as a burning cell
        self._mode = str(self.get_parameter("source_mode").value)
        if self._mode not in ("truth", "belief"):
            raise ValueError(f"source_mode must be 'truth' or 'belief', not {self._mode!r}")
        self._belief: HazardBelief | None = None

        name = str(self.get_parameter("scenario_name").value)
        if name:
            scn = scenario_by_name(name)
            cfg = scn.forecast_config
            apply_scenario_parameters(self, name, {
                "source_cells_xy": [v for src in scn.sources for v in (src.cell.x, src.cell.y)],
                "source_intensities": [float(src.intensity) for src in scn.sources],
                "forecast_steps": int(cfg.horizon_steps), "time_step_s": float(cfg.time_step_s),
                "spread_rate": float(cfg.spread_rate), "decay_rate": float(cfg.decay_rate),
                "uncertainty_growth": float(cfg.uncertainty_growth),
                # The live estimate advances one layer per update, so the period must equal the layer step.
                "update_period_s": float(cfg.time_step_s),
            })
        self._map_msg: OccupancyGrid | None = None
        self._grid = None
        self._current_risk: list[float] | None = None
        self._current_uncertainty: list[float] | None = None
        self._forecaster = HazardForecaster()
        self._ignited: dict[Cell, float] = {}
        # RViz "Publish Point" clicks start a simulated fire at that point (same validation);
        # empty string disables. Interactive testing input, not a sensor.
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        if self._mode == "truth":
            self.create_subscription(PointStamped, str(self.get_parameter("ignition_topic").value), self._on_ignition, 5)
            clicked = str(self.get_parameter("clicked_point_topic").value)
            if clicked:
                self.create_subscription(PointStamped, clicked, self._on_ignition, 5)
        else:
            self.create_subscription(OccupancyGrid, str(self.get_parameter("observation_topic").value),
                                     self._on_observation, 5)
            self._discovery_pub = self.create_publisher(PointStamped, "/aegisnav/hazard_discovered", 10)
        self._map_sub = self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value), self._on_map, MAP_QOS,
        )
        self._publisher = self.create_publisher(
            HazardForecast, str(self.get_parameter("forecast_topic").value), 1,
        )
        # RViz: current estimate (layer 0) and one predicted layer, kept visually distinct.
        self.declare_parameter("predicted_marker_step", 10)
        # Display threshold only (normalized risk); matches the lowest Gazebo tile band.
        self.declare_parameter("marker_min_risk", 0.25)
        # false for a second forecaster (e.g. the hidden truth when sensing is on), so RViz shows one field.
        self.declare_parameter("publish_markers", True)
        self._current_markers = self.create_publisher(MarkerArray, "/aegisnav/hazard_markers/current", 1)
        self._predicted_markers = self.create_publisher(MarkerArray, "/aegisnav/hazard_markers/predicted", 1)
        # Timer uses a minimum non-zero period, validated from the parameter.
        period = max(0.1, float(self.get_parameter("update_period_s").value))
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info("Hazard forecast adapter ready; waiting for a traversability map.")

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            grid = grid_from_occupancy_grid(msg, int(self.get_parameter("occupied_threshold").value))
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        # GridMap is immutable; equality checks dimensions, resolution, obstacle
        # set and origin, not just the array shape. Reset the simulated field if
        # any part of the map geometry or traversability changes.
        changed = self._grid != grid
        self._map_msg, self._grid = msg, grid
        if changed or self._current_risk is None:
            self._ignited.clear()
            self._current_risk = [0.0] * grid.size
            self._current_uncertainty = [0.12 if idx not in grid.blocked else 0.0 for idx in range(grid.size)]
            self.get_logger().info(f"Accepted map {grid.width}x{grid.height} at {grid.resolution_m:.3f} m/cell.")
            if self._mode == "belief":
                self._belief = HazardBelief(
                    grid, prior_uncertainty=float(self.get_parameter("belief_prior_uncertainty").value),
                    source_threshold=float(self.get_parameter("belief_source_threshold").value),
                )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_observation(self, msg: OccupancyGrid) -> None:
        if self._belief is None or self._grid is None or self._map_msg is None:
            return
        frame = self._map_msg.header.frame_id or "map"
        try:
            readings = observation_from_message(msg, self._grid, frame)
            discovered = self._belief.update(readings, self._now_s())
        except ValueError as exc:
            self.get_logger().error(f"Hazard observation rejected: {exc}")
            return
        for cell in discovered:
            wx, wy = self._grid.cell_center_world(cell)
            self.get_logger().warning(f"Hazard discovered by onboard sensor at cell ({cell.x}, {cell.y}).")
            point = PointStamped()
            point.header.frame_id, point.header.stamp = frame, msg.header.stamp
            point.point.x, point.point.y = wx, wy
            self._discovery_pub.publish(point)

    def _on_ignition(self, msg: PointStamped) -> None:
        if self._grid is None or self._map_msg is None:
            return
        frame = self._map_msg.header.frame_id or "map"
        intensity = float(self.get_parameter("ignition_intensity").value)
        if msg.header.frame_id != frame or not 0.0 <= intensity <= 1.0:
            self.get_logger().error(f"Ignoring ignition: frame {msg.header.frame_id!r} (map {frame!r}) or intensity {intensity}.")
            return
        cell = world_to_cell(self._grid, msg.point.x, msg.point.y)
        if cell is None or self._grid.is_blocked(cell):
            self.get_logger().error(f"Ignoring ignition at ({msg.point.x:.2f}, {msg.point.y:.2f}): outside map or in an occupied cell.")
            return
        if cell not in self._ignited:
            self._ignited[cell] = intensity
            self.get_logger().warning(f"Hazard ignition accepted at cell ({cell.x}, {cell.y}), intensity {intensity}.")

    def _sources(self):
        assert self._grid is not None
        cells = list(self.get_parameter("source_cells_xy").value)
        intensities = list(self.get_parameter("source_intensities").value)
        if len(cells) % 2:
            raise ValueError("source_cells_xy must be a flat even-length list [x0, y0, x1, y1, ...]")
        count = len(cells) // 2
        if not intensities:
            intensities = [0.9] * count
        if len(intensities) != count:
            raise ValueError("source_intensities must contain one value per source cell")
        sources = []
        for i in range(count):
            cell = Cell(int(cells[2 * i]), int(cells[2 * i + 1]))
            if not self._grid.contains(cell) or self._grid.is_blocked(cell):
                raise ValueError(f"Configured hazard source {cell} is outside the map or in an occupied cell")
            sources.append(HazardSource(cell, float(intensities[i]), f"configured_source_{i}"))
        sources.extend(HazardSource(c, i, "ignition") for c, i in self._ignited.items())
        return tuple(sources)

    def _tick(self) -> None:
        if self._grid is None or self._map_msg is None or self._current_risk is None or self._current_uncertainty is None:
            return
        try:
            config = ForecastConfig(
                horizon_steps=int(self.get_parameter("forecast_steps").value),
                time_step_s=float(self.get_parameter("time_step_s").value),
                spread_rate=float(self.get_parameter("spread_rate").value),
                decay_rate=float(self.get_parameter("decay_rate").value),
                uncertainty_growth=float(self.get_parameter("uncertainty_growth").value),
            )
            if self._mode == "belief":
                if self._belief is None:
                    return
                risk0, uncertainty0 = self._belief.layers(self._now_s())
                result = self._forecaster.forecast(self._grid, risk0, uncertainty0, self._belief.sources(), config)
            else:
                result = self._forecaster.forecast(self._grid, self._current_risk, self._current_uncertainty,
                                                   self._sources(), config)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Cannot produce hazard forecast: {exc}")
            return

        message = HazardForecast()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_msg.header.frame_id or "map"
        message.time_step_s = config.time_step_s
        for frame in result.frames:
            risk_msg = self._make_layer(frame.risk, message.header)
            uncertainty_msg = self._make_layer(frame.uncertainty, message.header)
            message.risk_layers.append(risk_msg)
            message.uncertainty_layers.append(uncertainty_msg)
        self._publisher.publish(message)
        if not bool(self.get_parameter("publish_markers").value):
            self._advance(result)
            return
        step = int(self.get_parameter("predicted_marker_step").value)
        if not 0 < step < len(result.frames):
            self.get_logger().error(f"predicted_marker_step {step} outside forecast horizon; predicted markers skipped.",
                                    throttle_duration_sec=10.0)
        else:
            self._publish_markers(result.frames[step], message.header, self._predicted_markers, predicted=True)
        self._publish_markers(result.frames[0], message.header, self._current_markers, predicted=False)
        self._advance(result)

    def _advance(self, result) -> None:
        # Advance the live estimate by one simulation time step. Persistent source
        # cells are re-applied by the model on every forecast cycle.
        self._current_risk = list(result.frames[min(1, len(result.frames) - 1)].risk)
        self._current_uncertainty = list(result.frames[min(1, len(result.frames) - 1)].uncertainty)

    def _make_layer(self, normalized, header):
        assert self._map_msg is not None and self._grid is not None
        layer = OccupancyGrid()
        layer.header = header
        layer.info = self._map_msg.info
        layer.data = [0 if idx in self._grid.blocked else max(0, min(100, round(value * 100))) for idx, value in enumerate(normalized)]
        return layer

    def _publish_markers(self, frame, header, publisher, *, predicted: bool) -> None:
        """One forecast layer for RViz. Current: warm filled tiles at floor level.
        Predicted: violet translucent tiles 0.25 m higher, so the two never look alike."""
        assert self._grid is not None
        ns = "aegisnav_hazard_predicted" if predicted else "aegisnav_hazard_current"
        array = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.ns = ns
        clear.id = 0
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for idx, risk in enumerate(frame.risk):
            if idx in self._grid.blocked or risk < float(self.get_parameter("marker_min_risk").value):
                continue
            cell = self._grid.cell(idx)
            wx, wy = self._grid.cell_center_world(cell)
            marker = Marker()
            marker.header = header
            marker.ns = ns
            marker.id = idx + 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.28 if predicted else 0.035
            marker.pose.orientation.w = 1.0
            marker.scale.x = self._grid.resolution_m * (0.7 if predicted else 0.88)
            marker.scale.y = self._grid.resolution_m * (0.7 if predicted else 0.88)
            marker.scale.z = 0.02 if predicted else 0.025
            if predicted:
                marker.color.r, marker.color.g, marker.color.b = 0.65, 0.25, 0.95
                marker.color.a = min(0.7, 0.10 + 0.60 * risk)
            else:
                marker.color.a = min(0.88, 0.18 + 0.70 * risk)
                if risk >= 0.80:
                    marker.color.r, marker.color.g, marker.color.b = 0.95, 0.10, 0.04
                elif risk >= 0.55:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.42, 0.02
                else:
                    marker.color.r, marker.color.g, marker.color.b = 0.95, 0.72, 0.06
            array.markers.append(marker)
        publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HazardForecastNode()
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
