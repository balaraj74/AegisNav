"""Deterministic, explicitly non-physical hazard propagation for simulation.

The model is an engineering abstraction for repeatable experiments. It is not a
validated fire-dynamics or toxic-gas dispersion solver and must never be
presented as one.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Iterable, Sequence

from .models import Cell, ForecastConfig, GridMap, HazardFrame, HazardSource


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


@dataclass(frozen=True, slots=True)
class ForecastResult:
    frames: tuple[HazardFrame, ...]

    def at_step(self, step: int) -> HazardFrame:
        if not self.frames:
            raise ValueError("Forecast contains no frames")
        return self.frames[min(max(step, 0), len(self.frames) - 1)]


class HazardForecaster:
    """Spreads normalized risk across free cells while respecting static walls."""

    def forecast(
        self,
        grid: GridMap,
        initial_risk: Sequence[float],
        initial_uncertainty: Sequence[float] | None = None,
        sources: Iterable[HazardSource] = (),
        config: ForecastConfig = ForecastConfig(),
    ) -> ForecastResult:
        if len(initial_risk) != grid.size:
            raise ValueError("initial_risk length must equal grid size")
        if any(not 0.0 <= float(v) <= 1.0 for v in initial_risk):
            raise ValueError("initial risk values must be in [0, 1]")
        if initial_uncertainty is None:
            uncertainty = [0.15] * grid.size
        else:
            if len(initial_uncertainty) != grid.size:
                raise ValueError("initial_uncertainty length must equal grid size")
            if any(not 0.0 <= float(v) <= 1.0 for v in initial_uncertainty):
                raise ValueError("initial uncertainty values must be in [0, 1]")
            uncertainty = [float(v) for v in initial_uncertainty]

        source_values: dict[int, float] = {}
        for source in sources:
            if not grid.contains(source.cell):
                raise ValueError(f"Hazard source outside grid: {source.cell}")
            if grid.is_blocked(source.cell):
                raise ValueError(f"Hazard source is placed inside a blocked cell: {source.cell}")
            source_values[grid.index(source.cell)] = max(source_values.get(grid.index(source.cell), 0.0), source.intensity)

        risk = [float(v) for v in initial_risk]
        # A configured source represents a currently active simulated hazard.
        # Seed it into layer 0 so the first published forecast does not briefly
        # claim that configured sources have zero current severity.
        for idx, intensity in source_values.items():
            risk[idx] = max(risk[idx], intensity)
        for idx in grid.blocked:
            risk[idx] = 0.0
            uncertainty[idx] = 0.0
        frames: list[HazardFrame] = []
        frames.append(HazardFrame(grid.width, grid.height, tuple(risk), tuple(uncertainty), 0, 0.0))

        # Per-grid tables (grids are immutable): orthogonal neighbour indices and the upwind index.
        neighbor_table = _orthogonal_neighbors(grid)
        wx, wy = config.wind_direction
        upwind_table = _upwind(grid, wx, wy) if (wx or wy) else None
        free_cells = [idx for idx in range(grid.size) if idx not in grid.blocked]
        spread, decay, growth, bias = config.spread_rate, config.decay_rate, config.uncertainty_growth, config.wind_bias

        for step in range(1, config.horizon_steps + 1):
            next_risk = risk.copy()
            next_uncertainty = uncertainty.copy()
            for idx in grid.blocked:
                next_risk[idx] = 0.0
                next_uncertainty[idx] = 0.0
            for idx in free_cells:
                neighbors = neighbor_table[idx]
                here = risk[idx]
                if neighbors:
                    values = [risk[n] for n in neighbors]
                    neighbor_max = max(values)
                    neighbor_mean = sum(values) / len(neighbors)
                else:
                    neighbor_max, neighbor_mean = 0.0, here

                # Diffusion toward local neighborhood risk, plus a directional
                # bias. This is a controllable simulation model, not physics.
                diffusion = spread * (0.60 * (neighbor_max - here) + 0.40 * (neighbor_mean - here))
                if upwind_table is not None:
                    upwind = upwind_table[idx]
                    if upwind >= 0:
                        diffusion += bias * max(0.0, risk[upwind] - here)
                candidate = _clamp(here + diffusion - decay * here)
                if idx in source_values:
                    candidate = max(candidate, source_values[idx])
                next_risk[idx] = candidate
                gradient = abs(candidate - here)
                next_uncertainty[idx] = _clamp(uncertainty[idx] + growth + 0.35 * gradient)

            risk, uncertainty = next_risk, next_uncertainty
            frames.append(HazardFrame(
                grid.width, grid.height, tuple(risk), tuple(uncertainty), step,
                step * config.time_step_s,
            ))
        return ForecastResult(tuple(frames))


@lru_cache(maxsize=16)
def _orthogonal_neighbors(grid: GridMap) -> tuple[tuple[int, ...], ...]:
    """Per cell: free orthogonal neighbour indices, in GridMap.neighbors order."""
    return tuple(tuple(grid.index(n) for n, _ in grid.neighbors(grid.cell(idx), diagonal=False))
                 for idx in range(grid.size))


@lru_cache(maxsize=16)
def _upwind(grid: GridMap, wx: int, wy: int) -> tuple[int, ...]:
    """Per cell: index of the free upwind cell (x - wx, y - wy), or -1."""
    out = []
    for idx in range(grid.size):
        c = grid.cell(idx)
        up = Cell(c.x - wx, c.y - wy)
        out.append(grid.index(up) if grid.contains(up) and not grid.is_blocked(up) else -1)
    return tuple(out)
