"""Seeded scenario randomization and result statistics for batch studies (ROS-free, stdlib only)."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from enum import StrEnum
from typing import Sequence

from .models import Cell
from .scenarios import Scenario


def randomized_ignitions(scn: Scenario, seed: int, *, count: int = 3, time_range_s: tuple[float, float] = (20.0, 240.0),
                         intensity: float = 0.95, min_start_dist_cells: int = 8,
                         min_goal_dist_cells: int = 3) -> tuple[tuple[float, Cell, float], ...]:
    """Deterministic simulated ignitions for one batch run: (time_s, cell, intensity), sorted by time.

    Cells are free cells that are not within `min_start_dist_cells` of the start/base (Chebyshev) or within
    `min_goal_dist_cells` of a mission goal, so a fire never blocks the robot at spawn or sits on a goal.
    The same (scenario, seed) always gives the same ignitions.
    """
    if count < 0 or not 0.0 <= intensity <= 1.0:
        raise ValueError("count must be >= 0 and intensity in [0, 1]")
    lo, hi = time_range_s
    if not 0.0 <= lo <= hi:
        raise ValueError("time_range_s must satisfy 0 <= low <= high")
    g = scn.grid
    keep_away = [(scn.start, min_start_dist_cells), (scn.base, min_start_dist_cells),
                 *[(goal, min_goal_dist_cells) for goal in scn.mission_goals]]
    occupied = {src.cell for src in scn.sources}
    candidates = [c for c in (g.cell(i) for i in range(g.size) if i not in g.blocked)
                  if c not in occupied and all(max(abs(c.x - k.x), abs(c.y - k.y)) > d for k, d in keep_away)]
    if len(candidates) < count:
        raise ValueError("not enough free cells for the requested ignitions")
    rng = random.Random(seed)
    chosen = rng.sample(candidates, count)
    times = sorted(round(rng.uniform(lo, hi), 1) for _ in range(count))
    return tuple((t, c, intensity) for t, c in zip(times, chosen))


class Outcome(StrEnum):
    COMPLETE = "complete"            # every inspection point visited, back at base
    PARTIAL = "partial"              # back at base, but at least one point was never reached
    REFUGE = "refuge"                # ended at a refuge by policy (e.g. degraded confidence)
    ABORTED = "aborted"              # the policy aborted (no safe route)
    STUCK = "stuck"                  # no progress: controller fault latch or stale inputs until timeout
    TIMEOUT = "timeout"              # mission clock ran out while still making progress


@dataclass(frozen=True, slots=True)
class RunFacts:
    goals_total: int
    goals_reached: int
    base_reached: bool
    ended_at_refuge: bool
    aborted: bool
    fault_latched: bool
    timed_out: bool


def classify_outcome(f: RunFacts) -> Outcome:
    """Single definition of the outcome taxonomy used by the batch reports."""
    if f.goals_total < 0 or not 0 <= f.goals_reached <= f.goals_total:
        raise ValueError("goals_reached must be within 0..goals_total")
    if f.base_reached:
        return Outcome.COMPLETE if f.goals_reached == f.goals_total else Outcome.PARTIAL
    if f.aborted:
        return Outcome.ABORTED
    if f.ended_at_refuge:
        return Outcome.REFUGE
    if f.fault_latched:
        return Outcome.STUCK
    return Outcome.TIMEOUT


def wilson_interval(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for a binomial proportion (95% by default). (0, 1) when n == 0."""
    if n < 0 or not 0 <= successes <= n:
        raise ValueError("need 0 <= successes <= n")
    if n == 0:
        return 0.0, 1.0
    p = successes / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def mean_sd(values: Sequence[float]) -> tuple[float, float]:
    """Sample mean and standard deviation (n-1); sd is 0 for fewer than two values."""
    if not values:
        raise ValueError("no values")
    m = sum(values) / len(values)
    if len(values) < 2:
        return m, 0.0
    return m, math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def paired_bootstrap_ci(a: Sequence[float], b: Sequence[float], *, iterations: int = 10_000, seed: int = 0,
                        alpha: float = 0.05) -> tuple[float, float, float]:
    """Mean of (a - b) over matched seeds with a percentile bootstrap interval: (mean, low, high).

    Pairs are runs of two modes on the same seed. Resampling is over seeds, with a fixed rng.
    """
    if len(a) != len(b) or not a:
        raise ValueError("a and b must be non-empty and the same length")
    diffs = [x - y for x, y in zip(a, b)]
    rng = random.Random(seed)
    n = len(diffs)
    means = sorted(sum(rng.choice(diffs) for _ in range(n)) / n for _ in range(iterations))
    return (sum(diffs) / n, means[int(alpha / 2 * iterations)], means[int((1 - alpha / 2) * iterations) - 1])
