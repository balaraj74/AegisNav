"""Conversions between ROS OccupancyGrid messages and AegisNav core models."""
from __future__ import annotations

import math
from typing import Any

from aegisnav_core.models import Cell, GridMap, HazardFrame


def origin_yaw(orientation: Any) -> float:
    """Return yaw while rejecting invalid and non-planar quaternions."""
    components = (orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(float(value)) for value in components):
        raise ValueError("Map-origin quaternion components must be finite")
    norm = math.sqrt(sum(float(value) ** 2 for value in components))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(f"Invalid map-origin quaternion norm: {norm:.4f}")
    if abs(float(orientation.x)) > 1e-4 or abs(float(orientation.y)) > 1e-4:
        raise ValueError("Roll/pitch-rotated grid origins are not supported by this adapter")
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def grid_from_occupancy_grid(message: Any, occupied_threshold: int = 50) -> GridMap:
    if not 1 <= int(occupied_threshold) <= 100:
        raise ValueError("occupied_threshold must be in [1, 100]")
    info = message.info
    if info.width <= 0 or info.height <= 0:
        raise ValueError("OccupancyGrid dimensions must be positive")
    if len(message.data) != info.width * info.height:
        raise ValueError(
            f"OccupancyGrid data length {len(message.data)} does not match dimensions {info.width}x{info.height}"
        )
    if not math.isfinite(float(info.resolution)) or info.resolution <= 0:
        raise ValueError("OccupancyGrid resolution must be finite and positive")
    origin_position = info.origin.position
    if not all(math.isfinite(float(value)) for value in (origin_position.x, origin_position.y, origin_position.z)):
        raise ValueError("OccupancyGrid origin coordinates must be finite")
    if abs(float(origin_position.z)) > 1e-4:
        raise ValueError("Non-planar OccupancyGrid origin height is not supported")
    yaw = origin_yaw(info.origin.orientation)
    if abs(yaw) > 1e-4:
        raise ValueError(f"Rotated OccupancyGrid origins are not supported by this adapter (yaw={yaw:.4f} rad)")
    invalid = [int(value) for value in message.data if int(value) < -1 or int(value) > 100]
    if invalid:
        raise ValueError(f"OccupancyGrid contains values outside the valid range [-1, 100]: {invalid[:5]}")
    blocked = frozenset(
        index for index, value in enumerate(message.data)
        if value < 0 or value >= occupied_threshold
    )
    return GridMap(
        width=info.width, height=info.height, resolution_m=float(info.resolution),
        blocked=blocked, origin_x_m=float(info.origin.position.x), origin_y_m=float(info.origin.position.y),
    )


def _check_layer(layer: Any, grid: GridMap, map_frame_id: str | None, what: str) -> None:
    """Fail closed unless an OccupancyGrid-shaped layer is aligned cell-for-cell with `grid`."""
    if layer.info.width != grid.width or layer.info.height != grid.height:
        raise ValueError(f"{what} dimensions do not match /map")
    layer_resolution = float(layer.info.resolution)
    if not math.isfinite(layer_resolution) or layer_resolution <= 0:
        raise ValueError(f"{what} layer resolution must be finite and positive")
    if abs(layer_resolution - grid.resolution_m) > 1e-6:
        raise ValueError(f"{what} resolution does not match /map")
    layer_origin = layer.info.origin.position
    if not all(math.isfinite(float(value)) for value in (layer_origin.x, layer_origin.y, layer_origin.z)):
        raise ValueError(f"{what} layer origin coordinates must be finite")
    if abs(float(layer_origin.x) - grid.origin_x_m) > 1e-5 or abs(float(layer_origin.y) - grid.origin_y_m) > 1e-5:
        raise ValueError(f"{what} origin does not match /map")
    if abs(origin_yaw(layer.info.origin.orientation)) > 1e-4:
        raise ValueError(f"Rotated {what} layers are not supported by this adapter")
    if abs(float(layer_origin.z)) > 1e-4:
        raise ValueError(f"Non-planar {what} layer origin height is not supported")
    if map_frame_id and layer.header.frame_id != map_frame_id:
        raise ValueError(f"{what} layer frame does not match /map frame")
    if len(layer.data) != grid.size:
        raise ValueError(f"{what} layer data length does not match /map")
    if layer.data and (min(layer.data) < -1 or max(layer.data) > 100):
        raise ValueError(f"{what} layer values must be in [-1, 100]")


def observation_from_message(message: Any, grid: GridMap, map_frame_id: str | None = None) -> dict[int, float]:
    """Hazard observation grid -> {cell index: observed risk in [0, 1]}; -1 cells were not seen."""
    _check_layer(message, grid, map_frame_id, "Hazard observation")
    return {idx: value / 100.0 for idx, value in enumerate(message.data) if value >= 0}


def observation_data(readings: dict[int, float], size: int) -> list[int]:
    """{cell index: risk in [0, 1]} -> OccupancyGrid data (0..100 observed, -1 not seen)."""
    data = [-1] * size
    for idx, risk in readings.items():
        if not 0 <= idx < size or not 0.0 <= risk <= 1.0:
            raise ValueError("observation reading outside the grid or [0, 1]")
        data[idx] = round(risk * 100)
    return data


# Value tables for int8 forecast cells: 0..100 -> value / 100; -1 (last element) -> unknown, which reads
# as no risk and full uncertainty.
_RISK_OF = tuple(v / 100.0 for v in range(101)) + (0.0,)
_UNCERTAINTY_OF = tuple(v / 100.0 for v in range(101)) + (1.0,)


def risk_frames_from_message(message: Any, grid: GridMap, map_frame_id: str | None = None) -> list[HazardFrame]:
    if not math.isfinite(float(message.time_step_s)) or float(message.time_step_s) <= 0:
        raise ValueError("HazardForecast time_step_s must be finite and positive")
    if map_frame_id and message.header.frame_id != map_frame_id:
        raise ValueError(f"Forecast frame {message.header.frame_id!r} does not match map frame {map_frame_id!r}")
    risk_layers = list(message.risk_layers)
    uncertainty_layers = list(message.uncertainty_layers)
    if not risk_layers or len(risk_layers) != len(uncertainty_layers):
        raise ValueError("HazardForecast must contain a non-empty, matching set of risk and uncertainty layers")
    frames: list[HazardFrame] = []
    for step, (risk_layer, uncertainty_layer) in enumerate(zip(risk_layers, uncertainty_layers, strict=True)):
        for layer in (risk_layer, uncertainty_layer):
            _check_layer(layer, grid, map_frame_id, "HazardForecast")
        # Lookup tables (index 0..100 = value/100, index -1 = unknown) instead of per-value branching.
        risk = tuple(map(_RISK_OF.__getitem__, risk_layer.data))
        uncertainty = tuple(map(_UNCERTAINTY_OF.__getitem__, uncertainty_layer.data))
        frames.append(HazardFrame(grid.width, grid.height, risk, uncertainty, step, step * float(message.time_step_s)))
    return frames


def world_to_cell(grid: GridMap, x_m: float, y_m: float) -> Cell | None:
    if not math.isfinite(float(x_m)) or not math.isfinite(float(y_m)):
        raise ValueError("World coordinates must be finite")
    x = math.floor((x_m - grid.origin_x_m) / grid.resolution_m)
    y = math.floor((y_m - grid.origin_y_m) / grid.resolution_m)
    cell = Cell(int(x), int(y))
    return cell if grid.contains(cell) else None


def snap_to_free_cell(grid: GridMap, x_m: float, y_m: float, max_offset_m: float) -> Cell | None:
    """Cell containing (x, y) if free; else the free cell whose boundary is nearest, if within max_offset_m.

    Localization noise can place a robot that is beside a wall a few millimetres
    inside the wall's grid cell. Snapping is bounded so that a pose genuinely
    inside an obstacle still fails closed (returns None).
    """
    if max_offset_m < 0 or not math.isfinite(max_offset_m):
        raise ValueError("max_offset_m must be finite and non-negative")
    cell = world_to_cell(grid, x_m, y_m)
    if cell is not None and not grid.is_blocked(cell):
        return cell
    reach = int(math.ceil(max_offset_m / grid.resolution_m)) + 1
    cx = math.floor((x_m - grid.origin_x_m) / grid.resolution_m)
    cy = math.floor((y_m - grid.origin_y_m) / grid.resolution_m)
    best: tuple[float, int, int] | None = None
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            cand = Cell(cx + dx, cy + dy)
            if not grid.contains(cand) or grid.is_blocked(cand):
                continue
            x0 = grid.origin_x_m + cand.x * grid.resolution_m
            y0 = grid.origin_y_m + cand.y * grid.resolution_m
            ddx = max(x0 - x_m, 0.0, x_m - (x0 + grid.resolution_m))
            ddy = max(y0 - y_m, 0.0, y_m - (y0 + grid.resolution_m))
            dist = math.hypot(ddx, ddy)
            if dist <= max_offset_m and (best is None or (dist, cand.y, cand.x) < best):
                best = (dist, cand.y, cand.x)
    return None if best is None else Cell(best[2], best[1])


def apply_scenario_parameters(node, name: str, values: dict) -> None:
    """Overwrite declared parameters with values derived from a named core scenario (single source)."""
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter(k, value=v) for k, v in values.items()])
    node.get_logger().info(f"Mission geometry and hazard sources from core scenario '{name}'.")
