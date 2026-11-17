"""Time-indexed risk-aware A* planning and mission-return feasibility checks."""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from functools import lru_cache
from itertools import count
from typing import Sequence

from .models import Cell, GridMap, HazardFrame, PlannerConfig, RoutePlan


@dataclass(frozen=True, slots=True)
class PlannerDiagnostics:
    reason: str
    expansions: int
    start_time_step: int


class RiskAwareAStar:
    """A* planner whose state includes forecast time, not just grid position.

    Travel time accumulates exactly (distance / nominal speed). A move is checked against every
    forecast layer from the one after its start time up to the first layer at or after its arrival,
    so risk is looked up no earlier than the robot can be there; the search state is
    (cell, that arrival layer). The forecast tail is held at its final layer after the supplied
    horizon; callers should cover the expected round-trip duration.
    """

    def plan(
        self,
        grid: GridMap,
        frames: Sequence[HazardFrame],
        start: Cell,
        goal: Cell,
        config: PlannerConfig = PlannerConfig(),
        *,
        start_time_step: int = 0,
        time_step_s: float = 1.0,
    ) -> RoutePlan:
        if not frames:
            return RoutePlan((), False, "missing_forecast")
        if time_step_s <= 0:
            raise ValueError("time_step_s must be positive")
        if not grid.contains(start) or not grid.contains(goal):
            return RoutePlan((), False, "start_or_goal_outside_map")
        if grid.is_blocked(start) or grid.is_blocked(goal):
            return RoutePlan((), False, "start_or_goal_blocked")
        for frame in frames:
            if frame.width != grid.width or frame.height != grid.height:
                raise ValueError("Forecast dimensions must match traversability grid")
        start_time_step = max(0, start_time_step)
        start_idx, goal_idx = grid.index(start), grid.index(goal)
        last_frame = len(frames) - 1

        near = (_clearance(grid, config.wall_clearance_cells)
                if config.wall_clearance_penalty_m > 0 or config.straight_near_obstacles else {})
        inflated = _clearance(grid, config.inflate_obstacles_cells) if config.inflate_obstacles_cells else {}
        adjacency = _adjacency(grid, config.diagonal_motion)
        risk_layers = [frame.risk for frame in frames]
        uncertainty_layers = [frame.uncertainty for frame in frames]
        resolution = grid.resolution_m
        zones = _standoff_zones(grid, frames, config.hazard_standoff_cells, config.hard_risk_threshold)
        if config.hazard_weight > 0 or config.uncertainty_weight > 0:
            rates = _min_rates(frames, config.hazard_weight, config.uncertainty_weight)
            heuristic = _cost_to_go(grid, adjacency, rates, goal_idx, config, near)
            if heuristic[start_idx] == math.inf:
                return RoutePlan((), False, "no_safe_path", expansions=0)
        else:
            # Distance-only planning (e.g. the evaluation baseline): keep the octile-distance heuristic,
            # whose tie-breaking between equal-length routes the baseline results were produced with.
            heuristic = _octile_to(grid, goal, config.diagonal_motion)
        start_state = (start_idx, min(start_time_step, last_frame))
        best_cost: dict[tuple[int, int], float] = {start_state: 0.0}
        # Exact elapsed time (s) of the best path to each state; states sharing a layer keep the
        # time of their cheapest path.
        arrival: dict[tuple[int, int], float] = {start_state: start_time_step * time_step_s}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        sequence = count()
        queue: list[tuple[float, float, int, tuple[int, int]]] = [
            (heuristic[start_idx], 0.0, next(sequence), start_state)
        ]
        expansions = 0
        terminal: tuple[int, int] | None = None

        while queue:
            _, cost_so_far, _, state = heapq.heappop(queue)
            if cost_so_far > best_cost.get(state, math.inf) + 1e-9:
                continue
            cell_idx = state[0]
            now_s = arrival[state]
            if cell_idx == goal_idx:
                terminal = state
                break
            expansions += 1
            if expansions > config.max_expansions:
                return RoutePlan((), False, "expansion_limit", expansions=expansions)

            for nxt_idx, length_cells in adjacency[cell_idx]:
                if nxt_idx in inflated and nxt_idx != goal_idx:
                    continue
                if heuristic[nxt_idx] == math.inf:
                    continue  # cannot reach the goal even with the most optimistic costs
                if config.straight_near_obstacles and length_cells > 1.0 and (cell_idx in near or nxt_idx in near):
                    continue
                step = _edge_cost(risk_layers, uncertainty_layers, now_s, nxt_idx, length_cells * resolution,
                                  config, time_step_s, clearance_weight=near.get(nxt_idx, 0),
                                  zones=zones, from_idx=cell_idx)
                if step is None:
                    continue
                next_state = (nxt_idx, step[1])
                new_cost = cost_so_far + step[2]
                if new_cost + 1e-9 >= best_cost.get(next_state, math.inf):
                    continue
                best_cost[next_state] = new_cost
                arrival[next_state] = step[0]
                parent[next_state] = state
                priority = new_cost + heuristic[nxt_idx]
                heapq.heappush(queue, (priority, new_cost, next(sequence), next_state))

        if terminal is None:
            return RoutePlan((), False, "no_safe_path", expansions=expansions)

        states = [terminal]
        while states[-1] != start_state:
            states.append(parent[states[-1]])
        states.reverse()
        cells = tuple(grid.cell(state[0]) for state in states)
        start_s = start_time_step * time_step_s
        scored = _score(grid, risk_layers, uncertainty_layers, cells, config, time_step_s, start_s, near, zones)
        if isinstance(scored, str):
            # A state's best path changed after its successors were expanded and the rescored route
            # now crosses the threshold: fail closed rather than return an unchecked route.
            return RoutePlan((), False, scored, expansions=expansions)
        start_risk = risk_layers[min(start_time_step, last_frame)][start_idx]
        start_uncertainty = uncertainty_layers[min(start_time_step, last_frame)][start_idx]
        estimated_time = scored.distance_m / config.nominal_speed_mps
        # Include the initial cell's immediate hazard/uncertainty for reporting.
        dwell = min(time_step_s * 0.5, estimated_time * 0.1)
        return RoutePlan(
            cells=cells, success=True, reason="ok", distance_m=scored.distance_m,
            estimated_time_s=estimated_time, hazard_exposure=scored.hazard_exposure + start_risk * dwell,
            uncertainty_exposure=scored.uncertainty_exposure + start_uncertainty * dwell,
            total_cost=best_cost[terminal], expansions=expansions,
            forecast_steps=math.ceil(estimated_time / time_step_s - 1e-9),
            max_risk_encountered=max(start_risk, scored.max_risk_encountered),
        )


def clearance_weights(grid: GridMap, radius: int) -> dict[int, int]:
    """Free cell index -> penalty multiple (radius - d + 1) for cells within Chebyshev distance d <= radius
    of a blocked cell. radius=1 reproduces "touches an obstacle" (multiple 1)."""
    out: dict[int, int] = {}
    for idx in grid.blocked:
        c = grid.cell(idx)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                n = Cell(c.x + dx, c.y + dy)
                if grid.contains(n) and not grid.is_blocked(n):
                    w = radius - max(abs(dx), abs(dy)) + 1
                    i = grid.index(n)
                    if w > out.get(i, 0):
                        out[i] = w
    return out


# Grids are immutable (frozen dataclass), so per-grid tables are cached across planning calls.
# The cached dicts/tuples are read-only by convention; callers must not mutate them.
@lru_cache(maxsize=16)
def _clearance(grid: GridMap, radius: int) -> dict[int, int]:
    return clearance_weights(grid, radius)


@lru_cache(maxsize=16)
def _adjacency(grid: GridMap, diagonal: bool) -> tuple[tuple[tuple[int, float], ...], ...]:
    """Per cell index: legal (neighbour index, length in cells), in GridMap.neighbors order."""
    return tuple(
        tuple((grid.index(nxt), length) for nxt, length in grid.neighbors(grid.cell(idx), diagonal=diagonal))
        if idx not in grid.blocked else ()
        for idx in range(grid.size)
    )


def _cost_to_go(grid: GridMap, adjacency, rate: Sequence[float], goal_idx: int,
                config: PlannerConfig, near: dict[int, int]) -> list[float]:
    """Admissible, consistent A* heuristic: exact cost-to-goal on a time-free relaxation.

    Each cell is charged its lowest hazard/uncertainty rate over all forecast layers, and the hard
    threshold, inflation and straight-near-obstacles rules are dropped, so every relaxed move costs
    no more than the same move in the real search (same distance and clearance penalty). Backward
    Dijkstra from the goal over the (symmetric) neighbour table. inf = cannot reach the goal at all.
    `rate` per cell comes from `_min_rates` (never more than any layer's weighted rate).
    """
    resolution, speed, penalty = grid.resolution_m, config.nominal_speed_mps, config.wall_clearance_penalty_m
    best = [math.inf] * grid.size
    best[goal_idx] = 0.0
    queue = [(0.0, goal_idx)]
    while queue:
        cost, v = heapq.heappop(queue)
        if cost > best[v]:
            continue
        # Moving u -> v costs: distance + travel time * rate(v) + clearance penalty of v.
        enter_v = near.get(v, 0) * penalty
        for u, length_cells in adjacency[v]:
            distance = length_cells * resolution
            new = cost + distance + distance / speed * rate[v] + enter_v
            if new < best[u]:
                best[u] = new
                heapq.heappush(queue, (new, u))
    return best


def _octile_to(grid: GridMap, goal: Cell, diagonal: bool) -> list[float]:
    """Straight-line (octile, or Manhattan without diagonals) distance in metres to `goal`; a lower bound."""
    out, k = [], math.sqrt(2.0) - 1.0
    for idx in range(grid.size):
        dx, dy = abs(idx % grid.width - goal.x), abs(idx // grid.width - goal.y)
        out.append(grid.resolution_m * (max(dx, dy) + k * min(dx, dy) if diagonal else dx + dy))
    return out


_RATE_CACHE: list = []  # [(frames, hw, uw, rates)]: one forecast is planned on several times per cycle


def _min_rates(frames: Sequence[HazardFrame], hw: float, uw: float) -> list[float]:
    """Per cell: hw * (min risk over layers) + uw * (min uncertainty over layers).

    Never more than min over layers of (hw * risk + uw * uncertainty), so heuristics built on it stay
    admissible. Cached for the most recent `frames` object (frames are immutable tuples of tuples).
    """
    if _RATE_CACHE and _RATE_CACHE[0][0] is frames and _RATE_CACHE[0][1:3] == (hw, uw):
        return _RATE_CACHE[0][3]
    if hw == 0 and uw == 0:
        rates = [0.0] * len(frames[0].risk)
    else:
        rates = [hw * r + uw * u for r, u in zip(map(min, zip(*(f.risk for f in frames))),
                                                  map(min, zip(*(f.uncertainty for f in frames))))]
    _RATE_CACHE[:] = [(frames, hw, uw, rates)]
    return rates


@lru_cache(maxsize=4096)
def _within(grid: GridMap, idx: int, radius: int) -> tuple[int, ...]:
    """Free cells reachable from `idx` in at most `radius` 8-neighbour steps through free space (idx included).
    Walls stop the spread, so a fire does not close the room on the other side of a wall."""
    seen, frontier = {idx}, [idx]
    for _ in range(radius):
        nxt = []
        for i in frontier:
            c = grid.cell(i)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    n = Cell(c.x + dx, c.y + dy)
                    if grid.contains(n) and not grid.is_blocked(n) and (j := grid.index(n)) not in seen:
                        seen.add(j)
                        nxt.append(j)
        frontier = nxt
    return tuple(sorted(seen))


_ZONE_CACHE: list = []  # [(frames, radius, threshold, zones)]: one forecast is planned on several times per cycle


def _standoff_zones(grid: GridMap, frames: Sequence[HazardFrame], radius: int, threshold: float):
    """Per forecast layer: frozenset of cell indices within `radius` cells of a cell at risk >= `threshold`.
    None when radius is 0 (feature off). Cached for the most recent `frames` object."""
    if radius <= 0:
        return None
    if _ZONE_CACHE and _ZONE_CACHE[0][0] is frames and _ZONE_CACHE[0][1:3] == (radius, threshold):
        return _ZONE_CACHE[0][3]
    memo: dict[tuple[int, ...], frozenset[int]] = {}
    zones = []
    for frame in frames:
        sources = tuple(i for i, v in enumerate(frame.risk) if v >= threshold)
        if sources not in memo:
            members: set[int] = set()
            for i in sources:
                members.update(_within(grid, i, radius))
            memo[sources] = frozenset(members)
        zones.append(memo[sources])
    _ZONE_CACHE[:] = [(frames, radius, threshold, zones)]
    return zones


def _edge_cost(
    risk_layers: Sequence[Sequence[float]], uncertainty_layers: Sequence[Sequence[float]], start_s: float,
    nxt_idx: int, distance_m: float, config: PlannerConfig, time_step_s: float, *, clearance_weight: int = 0,
    zones=None, from_idx: int = -1,
) -> tuple[float, int, float, float, float, float] | None:
    """(arrival time s, arrival layer, edge cost, mean risk, mean uncertainty, peak risk) for moving into
    cell `nxt_idx` starting at `start_s`, or None if any sampled layer crosses the hard threshold or the move
    enters a hazard standoff zone from outside it (`zones`, see PlannerConfig.hazard_standoff_cells).

    Single definition of the risk objective, shared by search and route evaluation. Sampled layers:
    from the first layer after the start time up to the first layer at or after the arrival time
    (never earlier than the robot can be in the cell); the tail is held at the last layer.
    """
    last_frame = len(risk_layers) - 1
    edge_duration_s = distance_m / config.nominal_speed_mps
    arrival_s = start_s + edge_duration_s
    first = min(math.floor(start_s / time_step_s + 1e-9) + 1, last_frame)
    arrival_layer = min(max(first, math.ceil(arrival_s / time_step_s - 1e-9)), last_frame)
    if arrival_layer == first:
        # One forecast sample (the usual case): no sample lists in the planner's innermost loop.
        risk = peak = risk_layers[first][nxt_idx]
        if risk >= config.hard_risk_threshold:
            return None
        if zones is not None and nxt_idx in zones[first] and from_idx not in zones[first]:
            return None
        uncertainty = uncertainty_layers[first][nxt_idx]
    else:
        layers = range(first, arrival_layer + 1)
        values = [risk_layers[layer][nxt_idx] for layer in layers]
        peak = max(values)
        if peak >= config.hard_risk_threshold:
            return None
        if zones is not None and any(nxt_idx in zones[k] and from_idx not in zones[k] for k in layers):
            return None
        risk = sum(values) / len(values)
        uncertainty = sum(uncertainty_layers[layer][nxt_idx] for layer in layers) / len(values)
    edge = distance_m + edge_duration_s * (config.hazard_weight * risk + config.uncertainty_weight * uncertainty)
    edge += clearance_weight * config.wall_clearance_penalty_m
    return arrival_s, arrival_layer, edge, risk, uncertainty, peak


@dataclass(frozen=True, slots=True)
class _Score:
    distance_m: float
    total_cost: float
    hazard_exposure: float
    uncertainty_exposure: float
    max_risk_encountered: float


def _score(grid: GridMap, risk_layers, uncertainty_layers, cells: Sequence[Cell], config: PlannerConfig,
           time_step_s: float, start_s: float, near: dict[int, int], zones=None) -> _Score | str:
    """Walk a legal-looking route with `_edge_cost`; a reason string if a step is rejected."""
    now_s, cost, distance, exposure, uncertainty_exposure, peak = start_s, 0.0, 0.0, 0.0, 0.0, 0.0
    for prev, cell in zip(cells, cells[1:]):
        length_m = math.hypot(cell.x - prev.x, cell.y - prev.y) * grid.resolution_m
        idx = grid.index(cell)
        step = _edge_cost(risk_layers, uncertainty_layers, now_s, idx, length_m, config, time_step_s,
                          clearance_weight=near.get(idx, 0), zones=zones, from_idx=grid.index(prev))
        if step is None:
            return "route_crosses_hard_threshold"
        duration_s = step[0] - now_s
        now_s = step[0]
        cost += step[2]
        exposure += step[3] * duration_s
        uncertainty_exposure += step[4] * duration_s
        peak = max(peak, step[5])
        distance += length_m
    return _Score(distance, cost, exposure, uncertainty_exposure, peak)


def evaluate_route(
    grid: GridMap, frames: Sequence[HazardFrame], cells: Sequence[Cell],
    config: PlannerConfig = PlannerConfig(), *, time_step_s: float = 1.0,
) -> RoutePlan:
    """Re-score an existing route under a (new) forecast with the planner's own objective.

    Returns success=False with a reason if the route is no longer legal (blocked
    cell, illegal or corner-cutting move) or any step crosses the hard threshold.
    """
    if not frames:
        return RoutePlan((), False, "missing_forecast")
    if not cells:
        return RoutePlan((), False, "empty_route")
    for frame in frames:
        if frame.width != grid.width or frame.height != grid.height:
            raise ValueError("Forecast dimensions must match traversability grid")
    if not grid.contains(cells[0]) or grid.is_blocked(cells[0]):
        return RoutePlan((), False, "route_start_blocked")
    near = (_clearance(grid, config.wall_clearance_cells)
            if config.wall_clearance_penalty_m > 0 or config.straight_near_obstacles else {})
    inflated = _clearance(grid, config.inflate_obstacles_cells) if config.inflate_obstacles_cells else {}
    risk_layers = [frame.risk for frame in frames]
    uncertainty_layers = [frame.uncertainty for frame in frames]
    if any(grid.index(c) in inflated for c in cells[1:-1]):
        return RoutePlan((), False, "route_step_illegal")
    for prev, cell in zip(cells, cells[1:]):
        legal = dict(grid.neighbors(prev, diagonal=config.diagonal_motion))
        if cell not in legal:
            return RoutePlan((), False, "route_step_illegal")
        if (config.straight_near_obstacles and legal[cell] > 1.0
                and (grid.index(prev) in near or grid.index(cell) in near)):
            return RoutePlan((), False, "route_step_illegal")
    zones = _standoff_zones(grid, frames, config.hazard_standoff_cells, config.hard_risk_threshold)
    scored = _score(grid, risk_layers, uncertainty_layers, cells, config, time_step_s, 0.0, near, zones)
    if isinstance(scored, str):
        return RoutePlan((), False, scored)
    peak = max(frames[0].at(cells[0])[0], scored.max_risk_encountered)
    return RoutePlan(tuple(cells), True, "ok", distance_m=scored.distance_m,
                     estimated_time_s=scored.distance_m / config.nominal_speed_mps, total_cost=scored.total_cost,
                     hazard_exposure=scored.hazard_exposure, max_risk_encountered=peak)
