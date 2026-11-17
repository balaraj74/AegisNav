"""Typed domain models shared by the simulator-independent AegisNav core.

This module intentionally depends only on Python's standard library. Keep ROS
messages and simulator-specific types in adapter packages, never here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from math import isfinite, sqrt
from typing import Iterable


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


@dataclass(frozen=True, slots=True)
class Cell:
    x: int
    y: int


@dataclass(frozen=True, slots=True)
class GridMap:
    """A 2-D traversability grid; coordinates are integer cell indices."""

    width: int
    height: int
    resolution_m: float = 1.0
    blocked: frozenset[int] = field(default_factory=frozenset)
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Grid width and height must be positive")
        if not isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("Grid resolution must be finite and positive")
        if not isfinite(self.origin_x_m) or not isfinite(self.origin_y_m):
            raise ValueError("Grid origin coordinates must be finite")
        total = self.width * self.height
        if any(index < 0 or index >= total for index in self.blocked):
            raise ValueError("Blocked cell index falls outside the grid")

    @property
    def size(self) -> int:
        return self.width * self.height

    def contains(self, cell: Cell) -> bool:
        return 0 <= cell.x < self.width and 0 <= cell.y < self.height

    def index(self, cell: Cell) -> int:
        if not self.contains(cell):
            raise IndexError(f"Cell {cell} falls outside {self.width}x{self.height} grid")
        return cell.y * self.width + cell.x

    def cell(self, index: int) -> Cell:
        if index < 0 or index >= self.size:
            raise IndexError(f"Cell index {index} falls outside grid")
        return Cell(index % self.width, index // self.width)

    def is_blocked(self, cell: Cell) -> bool:
        return self.index(cell) in self.blocked

    def neighbors(self, cell: Cell, *, diagonal: bool = True) -> Iterable[tuple[Cell, float]]:
        """Yield legal neighbor cells and movement length in cell units.

        Diagonal corner-cutting is forbidden: both orthogonal cells beside a
        diagonal move must be traversable.
        """
        steps = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0))
        if diagonal:
            steps += ((-1, -1, sqrt(2)), (-1, 1, sqrt(2)), (1, -1, sqrt(2)), (1, 1, sqrt(2)))
        for dx, dy, distance in steps:
            nxt = Cell(cell.x + dx, cell.y + dy)
            if not self.contains(nxt) or self.is_blocked(nxt):
                continue
            if dx and dy:
                if self.is_blocked(Cell(cell.x + dx, cell.y)) or self.is_blocked(Cell(cell.x, cell.y + dy)):
                    continue
            yield nxt, distance

    def cell_center_world(self, cell: Cell) -> tuple[float, float]:
        return (
            self.origin_x_m + (cell.x + 0.5) * self.resolution_m,
            self.origin_y_m + (cell.y + 0.5) * self.resolution_m,
        )


@dataclass(frozen=True, slots=True)
class HazardFrame:
    """Normalized hazard risk and epistemic uncertainty for one forecast step."""

    width: int
    height: int
    risk: tuple[float, ...]
    uncertainty: tuple[float, ...]
    step_index: int = 0
    time_s: float = 0.0

    def __post_init__(self) -> None:
        expected = self.width * self.height
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Hazard frame dimensions must be positive")
        if len(self.risk) != expected or len(self.uncertainty) != expected:
            raise ValueError("Risk and uncertainty arrays must match frame dimensions")
        if any(not 0.0 <= value <= 1.0 for value in self.risk):
            raise ValueError("Risk values must be normalized to [0, 1]")
        if any(not 0.0 <= value <= 1.0 for value in self.uncertainty):
            raise ValueError("Uncertainty values must be normalized to [0, 1]")
        if self.step_index < 0 or self.time_s < 0:
            raise ValueError("Forecast step and time must be non-negative")

    def at(self, cell: Cell) -> tuple[float, float]:
        if not (0 <= cell.x < self.width and 0 <= cell.y < self.height):
            raise IndexError(f"Cell {cell} falls outside hazard frame")
        idx = cell.y * self.width + cell.x
        return self.risk[idx], self.uncertainty[idx]


@dataclass(frozen=True, slots=True)
class HazardSource:
    cell: Cell
    intensity: float = 1.0
    label: str = "unspecified"

    def __post_init__(self) -> None:
        if not 0.0 <= self.intensity <= 1.0:
            raise ValueError("Hazard source intensity must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class ForecastConfig:
    horizon_steps: int = 20
    time_step_s: float = 1.0
    spread_rate: float = 0.22
    decay_rate: float = 0.015
    uncertainty_growth: float = 0.025
    wind_direction: tuple[int, int] = (0, 0)
    wind_bias: float = 0.15

    def __post_init__(self) -> None:
        if self.horizon_steps < 1:
            raise ValueError("Forecast horizon_steps must be at least 1")
        if self.time_step_s <= 0:
            raise ValueError("time_step_s must be positive")
        for name, value in (("spread_rate", self.spread_rate), ("decay_rate", self.decay_rate), ("uncertainty_growth", self.uncertainty_growth), ("wind_bias", self.wind_bias)):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.wind_direction not in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
            raise ValueError("wind_direction must be cardinal or (0, 0)")


@dataclass(frozen=True, slots=True)
class PlannerConfig:
    hazard_weight: float = 12.0
    uncertainty_weight: float = 2.5
    hard_risk_threshold: float = 0.92
    diagonal_motion: bool = True
    nominal_speed_mps: float = 0.35
    max_expansions: int = 200_000
    # Metres of equivalent path length added for entering a cell that touches an
    # obstacle (8-neighbourhood). Static traversability preference, not hazard risk:
    # keeps the robot body off walls where a wider lane exists.
    wall_clearance_penalty_m: float = 0.3
    # Cells within this Chebyshev distance of an obstacle are penalised, (radius - d + 1) x penalty.
    # 1 = only cells touching an obstacle (small robot); 2 suits a ~1 m robot on 0.5 m cells.
    wall_clearance_cells: int = 1
    # Forbid diagonal moves into/out of cells within wall_clearance_cells of an obstacle, so a long
    # robot passes doorways and wall-side lanes straight instead of turning inside them.
    straight_near_obstacles: bool = False
    # Hard configuration-space margin: cells within this Chebyshev distance of an obstacle are not
    # traversable (except as start or goal). 0 = point robot; 1 suits a ~0.6 m-wide robot on 0.5 m cells.
    inflate_obstacles_cells: int = 0

    def __post_init__(self) -> None:
        if self.hazard_weight < 0 or self.uncertainty_weight < 0:
            raise ValueError("Planner weights must be non-negative")
        if not isfinite(self.wall_clearance_penalty_m) or self.wall_clearance_penalty_m < 0:
            raise ValueError("wall_clearance_penalty_m must be finite and non-negative")
        if not 1 <= self.wall_clearance_cells <= 5:
            raise ValueError("wall_clearance_cells must be in [1, 5]")
        if not 0 <= self.inflate_obstacles_cells <= 3:
            raise ValueError("inflate_obstacles_cells must be in [0, 3]")
        if not 0.0 < self.hard_risk_threshold <= 1.0:
            raise ValueError("hard_risk_threshold must be in (0, 1]")
        if self.nominal_speed_mps <= 0:
            raise ValueError("nominal_speed_mps must be positive")
        if self.max_expansions < 1:
            raise ValueError("max_expansions must be positive")


@dataclass(frozen=True, slots=True)
class RoutePlan:
    cells: tuple[Cell, ...]
    success: bool
    reason: str
    distance_m: float = 0.0
    estimated_time_s: float = 0.0
    hazard_exposure: float = 0.0
    uncertainty_exposure: float = 0.0
    total_cost: float = 0.0
    expansions: int = 0
    forecast_steps: int = 0
    max_risk_encountered: float = 0.0

    @property
    def duration_steps(self) -> int:
        """Forecast layers consumed by the estimated route, rounded up per move."""
        return self.forecast_steps if self.success else 0


class MissionAction(StrEnum):
    CONTINUE_TO_TARGET = "continue_to_target"
    REPLAN = "replan"
    SLOW_AND_REASSESS = "slow_and_reassess"
    RETURN_TO_BASE = "return_to_base"
    SEEK_REFUGE = "seek_refuge"
    ABORT = "abort"


@dataclass(frozen=True, slots=True)
class MissionAssessment:
    action: MissionAction
    reason_code: str
    explanation: str
    outbound: RoutePlan | None = None
    return_route: RoutePlan | None = None
    refuge_route: RoutePlan | None = None
    confidence: float = 1.0
    safe_return_feasible: bool = False
    forecast_horizon_sufficient: bool = True

    def __post_init__(self) -> None:
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")

    @property
    def execution_route(self) -> RoutePlan | None:
        """Route the robot should execute for this decision, or None to stop.

        Single definition shared by every consumer so the ROS node and the
        closed-loop runner cannot diverge. ABORT never yields a route.
        """
        if self.action is MissionAction.CONTINUE_TO_TARGET:
            route = self.outbound
        elif self.action in (MissionAction.SEEK_REFUGE, MissionAction.SLOW_AND_REASSESS):
            route = self.refuge_route
        elif self.action is MissionAction.RETURN_TO_BASE:
            route = self.return_route
        else:
            return None
        return route if route is not None and route.success else None
