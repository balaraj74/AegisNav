"""Simulated onboard hazard sensing and the robot's hazard belief (ROS-free, stdlib only).

The robot does not know the hazard field. A simulated sensor reports the true current risk only in grid
cells within range and line of sight (walls occlude); everything else stays unknown. The belief keeps the
last observation per cell and turns observed burning cells into forecast sources, so the planner forecasts
from what the robot has seen. The sensor model (range, noise, occlusion on the planning grid) is a
simulation assumption, not a model of a specific thermal or gas sensor.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from .models import Cell, GridMap, HazardSource


def _line(a: Cell, b: Cell) -> list[Cell]:
    """Cells on the Bresenham line from a to b, both ends included."""
    x0, y0, x1, y1 = a.x, a.y, b.x, b.y
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err, out = dx + dy, []
    while True:
        out.append(Cell(x0, y0))
        if x0 == x1 and y0 == y1:
            return out
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def visible_cells(grid: GridMap, origin: Cell, range_m: float, *,
                  obscured: frozenset[int] | set[int] = frozenset()) -> frozenset[int]:
    """Indices of cells within `range_m` (cell-centre distance) and line of sight of `origin`.

    The first blocked cell on a ray is visible (a wall, or a victim lying on a blocked cell); nothing
    behind it is. A diagonal step between two blocked cells is treated as occluded. `obscured` cells
    (e.g. dense smoke) are seen but block the view beyond them, except the robot's own cell.
    """
    if not grid.contains(origin):
        raise ValueError(f"Sensor origin {origin} is outside the grid")
    if not math.isfinite(range_m) or range_m < 0:
        raise ValueError("range_m must be finite and non-negative")
    reach = int(range_m / grid.resolution_m)
    out = {grid.index(origin)}
    for y in range(max(0, origin.y - reach), min(grid.height, origin.y + reach + 1)):
        for x in range(max(0, origin.x - reach), min(grid.width, origin.x + reach + 1)):
            target = Cell(x, y)
            if math.hypot(x - origin.x, y - origin.y) * grid.resolution_m > range_m:
                continue
            ray = _line(origin, target)
            for prev, cell in zip(ray, ray[1:]):
                if (cell.x != prev.x and cell.y != prev.y and grid.is_blocked(Cell(cell.x, prev.y))
                        and grid.is_blocked(Cell(prev.x, cell.y))):
                    break  # squeezing between two blocked cells
                idx = grid.index(cell)
                out.add(idx)
                if grid.is_blocked(cell) or idx in obscured:
                    break
    return frozenset(out)


def observe(truth_risk: Sequence[float], visible: Iterable[int], *, noise_sigma: float = 0.0,
            rng: random.Random | None = None) -> dict[int, float]:
    """Sensor reading: true current risk in each visible cell, plus clamped Gaussian noise (seeded rng)."""
    if noise_sigma < 0:
        raise ValueError("noise_sigma must be non-negative")
    if noise_sigma > 0 and rng is None:
        raise ValueError("a seeded rng is required when noise_sigma > 0")
    out: dict[int, float] = {}
    for idx in sorted(visible):  # sorted: the noise sequence must not depend on set iteration order
        value = float(truth_risk[idx])
        if noise_sigma > 0:
            value += rng.gauss(0.0, noise_sigma)
        out[idx] = max(0.0, min(1.0, value))
    return out


@dataclass
class HazardBelief:
    """What the robot knows about the hazard field.

    Unobserved cells: risk `prior_risk`, uncertainty `prior_uncertainty` (unknown is not the same as safe).
    Observed cells: the last reading; uncertainty grows from `observed_uncertainty` by
    `uncertainty_growth_per_s` with the observation's age. Observed cells at or above `source_threshold`
    become forecast sources at the observed intensity. A *discovery* is a burning cell with no burning
    cell already known within `region_radius_cells` (a new hazard region, not the spread of a known one).
    "Known" includes every cell ever seen burning, so sensor noise around the threshold does not re-report
    the same fire; forecast sources still follow the latest reading.
    """

    grid: GridMap
    prior_risk: float = 0.0
    prior_uncertainty: float = 0.6
    observed_uncertainty: float = 0.1
    uncertainty_growth_per_s: float = 0.01
    source_threshold: float = 0.5
    region_radius_cells: int = 2
    _readings: dict[int, tuple[float, float]] = field(default_factory=dict)  # idx -> (risk, time_s)
    _ever_burning: set[int] = field(default_factory=set)

    def __post_init__(self) -> None:
        for name in ("prior_risk", "prior_uncertainty", "observed_uncertainty", "uncertainty_growth_per_s"):
            if not 0.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if not 0.0 < self.source_threshold <= 1.0:
            raise ValueError("source_threshold must be in (0, 1]")
        if self.region_radius_cells < 0:
            raise ValueError("region_radius_cells must be non-negative")

    def update(self, readings: dict[int, float], time_s: float) -> list[Cell]:
        """Record readings; return the cells that reveal a new hazard region (see class docstring)."""
        if any(not 0.0 <= risk <= 1.0 for risk in readings.values()):
            raise ValueError("readings must be in [0, 1]")
        known = self._ever_burning
        discovered = []
        for idx in sorted(readings):
            risk = readings[idx]
            if not 0 <= idx < self.grid.size or idx in self.grid.blocked:
                continue
            if risk >= self.source_threshold and idx not in known:
                if not self._near(idx, known):
                    discovered.append(self.grid.cell(idx))
                known.add(idx)
            self._readings[idx] = (risk, time_s)
        return discovered

    def _near(self, idx: int, cells: set[int]) -> bool:
        r, w = self.region_radius_cells, self.grid.width
        x, y = idx % w, idx // w
        return any(abs(c % w - x) <= r and abs(c // w - y) <= r for c in cells)

    def observed(self) -> frozenset[int]:
        return frozenset(self._readings)

    def sources(self) -> tuple[HazardSource, ...]:
        return tuple(HazardSource(self.grid.cell(idx), risk, "observed")
                     for idx, (risk, _) in sorted(self._readings.items()) if risk >= self.source_threshold)

    def layers(self, time_s: float) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """(risk, uncertainty) per cell at `time_s`, the forecaster's initial state."""
        risk = [self.prior_risk] * self.grid.size
        uncertainty = [self.prior_uncertainty] * self.grid.size
        for idx, (value, seen_s) in self._readings.items():
            risk[idx] = value
            age = max(0.0, time_s - seen_s)
            uncertainty[idx] = min(1.0, self.observed_uncertainty + self.uncertainty_growth_per_s * age)
        for idx in self.grid.blocked:
            risk[idx], uncertainty[idx] = 0.0, 0.0
        return tuple(risk), tuple(uncertainty)


def detected(visible: frozenset[int], grid: GridMap, targets: Iterable[Cell]) -> list[Cell]:
    """Targets (e.g. victims) whose cell is visible, in input order."""
    return [t for t in targets if grid.contains(t) and grid.index(t) in visible]


def estimate_confidence(visible_count: int, clear_view_count: int, localization_std_m: float, *,
                        max_localization_std_m: float = 1.0) -> float:
    """Perception confidence in [0, 1] from observable quantities (a heuristic, not calibrated).

    min(view quality, localization quality):
      view quality = cells visible now / cells visible in clear air from the same pose;
      localization quality = 1 - position standard deviation / max_localization_std_m (clamped).
    Default scale 1.0 m, tied to the 0.5 m planning cell: with MissionPolicy's thresholds, position
    doubt of about one cell (sigma ~0.45 m) is "degraded" (< 0.55) and sigma >= 0.7 m is "critical"
    (<= 0.30). Normal AMCL tracking measured in the container (sigma 0.10-0.18 m) reads 0.82-0.90.
    """
    if visible_count < 0 or clear_view_count <= 0 or visible_count > clear_view_count:
        raise ValueError("need 0 <= visible_count <= clear_view_count and clear_view_count > 0")
    if not math.isfinite(localization_std_m) or localization_std_m < 0:
        raise ValueError("localization_std_m must be finite and non-negative")
    if max_localization_std_m <= 0:
        raise ValueError("max_localization_std_m must be positive")
    view = visible_count / clear_view_count
    localization = max(0.0, 1.0 - localization_std_m / max_localization_std_m)
    return min(view, localization)
