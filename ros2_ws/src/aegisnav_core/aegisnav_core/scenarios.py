"""Deterministic reference scenarios used by the CLI and regression tests."""
from __future__ import annotations

from dataclasses import dataclass

from .models import Cell, ForecastConfig, GridMap, HazardSource


@dataclass(frozen=True, slots=True)
class Scenario:
    scenario_id: str
    grid: GridMap
    start: Cell
    target: Cell
    base: Cell
    refuges: tuple[Cell, ...]
    sources: tuple[HazardSource, ...]
    initial_risk: tuple[float, ...]
    initial_uncertainty: tuple[float, ...]
    forecast_config: ForecastConfig
    # Mission waypoints in order (free cells); empty means (target,). Base follows the last one.
    waypoints: tuple[Cell, ...] = ()
    # Blocked cells occupied by victims and by props (kind is a rendering hint only).
    victims: tuple[Cell, ...] = ()
    props: tuple[tuple[Cell, str], ...] = ()
    # Narrowest passage (cells) the robot this scenario is designed for can use; checked by tests.
    min_passage_cells: int = 2
    # Scripted demo events (simulated inputs): (time_s, cell, intensity) and (time_s, confidence).
    scripted_ignitions: tuple[tuple[float, Cell, float], ...] = ()
    scripted_confidence: tuple[tuple[float, float], ...] = ()

    @property
    def mission_goals(self) -> tuple[Cell, ...]:
        return self.waypoints or (self.target,)


def corridor_scenario() -> Scenario:
    """Two routes around a wall; a spreading hazard threatens the lower gate (y=3..4)."""
    width, height = 24, 16
    blocked: set[int] = set()
    # Perimeter walls model a bounded indoor rescue test zone.
    for x in range(width):
        blocked.add(x)
        blocked.add((height - 1) * width + x)
    for y in range(height):
        blocked.add(y * width)
        blocked.add(y * width + (width - 1))
    # Interior divider with two gates. This forces an explicit route choice.
    # Every opening is two cells (0.8 m): one-cell (0.4 m) gaps were not
    # traversable by the 0.23 m-wide simulated robot under Nav2's local
    # controller (observed: "Controller patience exceeded" in Gazebo).
    for y in range(1, height - 1):
        if y not in (3, 4, 11, 12):
            blocked.add(y * width + 11)
    # Short interior partitions create a more legible route topology.
    for x in range(4, 9):
        if x not in (6, 7):
            blocked.add(7 * width + x)
    for x in range(14, 20):
        if x not in (16, 17):
            blocked.add(8 * width + x)
    grid = GridMap(width, height, resolution_m=0.4, blocked=frozenset(blocked), origin_x_m=-4.8, origin_y_m=-3.2)
    sources = (HazardSource(Cell(11, 3), 0.94, "lower-gate-fire"), HazardSource(Cell(9, 4), 0.42, "smoke-front"))
    initial = [0.0] * grid.size
    uncertainty = [0.12] * grid.size
    for source in sources:
        initial[grid.index(source.cell)] = source.intensity
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                cell = Cell(source.cell.x + dx, source.cell.y + dy)
                if grid.contains(cell) and not grid.is_blocked(cell):
                    idx = grid.index(cell)
                    initial[idx] = max(initial[idx], source.intensity * (0.42 if dx or dy else 1.0))
                    uncertainty[idx] = min(1.0, uncertainty[idx] + 0.24)
    return Scenario(
        scenario_id="corridor_hazard_spread_v2", grid=grid,
        start=Cell(2, 2), target=Cell(21, 13), base=Cell(2, 2),
        refuges=(Cell(2, 12), Cell(21, 2), Cell(6, 12)),
        sources=sources, initial_risk=tuple(initial), initial_uncertainty=tuple(uncertainty),
        forecast_config=ForecastConfig(horizon_steps=120, time_step_s=1.0, spread_rate=0.24, decay_rate=0.008, uncertainty_growth=0.018),
    )


def _rect(width: int, x0: int, y0: int, x1: int, y1: int) -> set[int]:
    return {y * width + x for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)}


def rescue_building_scenario() -> Scenario:
    """20 m x 14 m building (0.5 m cells) for a Husky-class robot: corridor, 7 rooms, 2 victims.

    Every doorway is 4 cells (2.0 m); rooms connect through side doors, so blocked
    corridors leave alternative routes. A fire burns in the corridor and a gas leak in
    the south-middle room (same synthetic spread model; the label only affects rendering).
    """
    w, h = 40, 28
    blocked: set[int] = set()
    blocked |= _rect(w, 0, 0, w - 1, 0) | _rect(w, 0, h - 1, w - 1, h - 1)
    blocked |= _rect(w, 0, 0, 0, h - 1) | _rect(w, w - 1, 0, w - 1, h - 1)
    blocked |= _rect(w, 1, 16, w - 2, 16) | _rect(w, 1, 11, w - 2, 11)      # corridor walls (y 12..15 free)
    for x in (10, 20, 30):
        blocked |= _rect(w, x, 17, x, h - 2)                                 # north room walls
    for x in (13, 26):
        blocked |= _rect(w, x, 1, x, 10)                                      # south room walls
    # 4-cell (2.0 m) breaches: the 1.0 x 0.57 m robot must be able to enter them at an angle.
    doors = [(4, 16, 7, 16), (14, 16, 17, 16), (23, 16, 26, 16), (33, 16, 36, 16),   # north rooms <-> corridor
             (5, 11, 8, 11), (17, 11, 20, 11), (31, 11, 34, 11),                     # south rooms <-> corridor
             (10, 21, 10, 24), (20, 21, 20, 24), (30, 19, 30, 22),                    # north side doors
             (13, 4, 13, 7), (26, 3, 26, 6)]                                          # south side doors
    for d in doors:
        blocked -= _rect(w, *d)
    props = []
    for (x0, y0, x1, y1), kind in (((11, 25, 12, 26), "rubble"), ((37, 17, 38, 19), "pallets"),
                                    ((1, 8, 2, 10), "cinder"), ((24, 1, 25, 2), "barrels"),
                                    ((27, 9, 28, 10), "barrels"), ((21, 15, 22, 15), "rubble")):
        for idx in _rect(w, x0, y0, x1, y1):
            props.append((Cell(idx % w, idx // w), kind))
    victims = (Cell(29, 26), Cell(38, 1))
    blocked |= {c.y * w + c.x for c, _ in props} | {c.y * w + c.x for c in victims}
    grid = GridMap(w, h, resolution_m=0.5, blocked=frozenset(blocked), origin_x_m=-10.0, origin_y_m=-7.0)
    sources = (HazardSource(Cell(19, 13), 0.95, "fire"), HazardSource(Cell(20, 6), 0.8, "gas"))
    initial = [0.0] * grid.size
    uncertainty = [0.12] * grid.size
    for src in sources:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                c = Cell(src.cell.x + dx, src.cell.y + dy)
                if grid.contains(c) and not grid.is_blocked(c):
                    i = grid.index(c)
                    initial[i] = max(initial[i], src.intensity * (0.42 if dx or dy else 1.0))
                    uncertainty[i] = min(1.0, uncertainty[i] + 0.24)
    waypoints = (Cell(27, 24), Cell(36, 3))
    return Scenario(
        scenario_id="rescue_building_v1", grid=grid,
        start=Cell(3, 3), target=waypoints[0], base=Cell(3, 3),
        refuges=(Cell(3, 24), Cell(16, 3), Cell(35, 24)),
        sources=sources, initial_risk=tuple(initial), initial_uncertainty=tuple(uncertainty),
        # 2 s layers x 90 = 180 s, covering the longest leg plus its return at 0.7 m/s.
        # Slower spread / faster decay than the corridor scenario: with 2 s layers the corridor values
        # filled most of the building within ~2 minutes (observed); synthetic-model parameters.
        forecast_config=ForecastConfig(horizon_steps=90, time_step_s=2.0, spread_rate=0.15, decay_rate=0.02, uncertainty_growth=0.018),
        waypoints=waypoints, victims=victims, props=tuple(props), min_passage_cells=3,
        # Demo script (mission time from the first localized pose): burning debris across the hall just
        # beyond the west door, beside the first-leg route, while the robot is still in the west
        # corridor; it can detour through the middle room (test_scenarios checks both; too late after
        # ~12 s). On the return leg the simulated perception confidence degrades. Observed headless
        # without the drop: victim 2 at ~86 s, base at ~125 s.
        scripted_ignitions=((8.0, Cell(5, 21), 0.97), (8.0, Cell(6, 21), 0.97)),
        scripted_confidence=((105.0, 0.42),),
    )


def rescue_compound_scenario() -> Scenario:
    """36 m x 22 m compound (0.5 m cells): the rescue building, a north courtyard, an east yard and a warehouse.

    The building occupies the same cells and world coordinates as `rescue_building_scenario`, with two new
    exits: its corridor opens east into the yard, and its north-west room opens into the courtyard. Five
    inspection points (victim approaches and a yard sector) are visited in order, then base. Every passage
    is at least 3 cells (1.5 m) wide.
    """
    building = rescue_building_scenario()
    bw = building.grid.width
    w, h = 72, 44
    blocked = {(i // bw) * w + i % bw for i in building.grid.blocked}
    blocked |= _rect(w, 0, h - 1, w - 1, h - 1) | _rect(w, 0, 0, w - 1, 0)            # compound perimeter
    blocked |= _rect(w, 0, 0, 0, h - 1) | _rect(w, w - 1, 0, w - 1, h - 1)
    blocked -= _rect(w, 39, 12, 39, 15)                                                 # building east exit
    blocked -= _rect(w, 2, 27, 5, 27)                                                   # building north exit
    # Warehouse in the north-east corner: the compound perimeter is its north and east wall.
    blocked |= _rect(w, 46, 24, w - 1, 24) | _rect(w, 46, 24, 46, h - 1)
    blocked -= _rect(w, 50, 24, 53, 24) | _rect(w, 62, 24, 65, 24) | _rect(w, 46, 34, 46, 37)  # warehouse doors
    for y in (28, 32, 36):
        blocked |= _rect(w, 50, y, 65, y)                                               # shelving rows
    props = [(Cell(c.x, c.y), kind) for c, kind in building.props]
    for (x0, y0, x1, y1), kind in (((12, 33, 14, 35), "rubble"), ((24, 31, 26, 32), "rubble"),
                                    ((55, 8, 56, 9), "barrels"), ((62, 14, 64, 15), "pallets"),
                                    ((43, 18, 44, 19), "cinder"), ((55, 40, 56, 42), "pallets"), ((34, 41, 34, 42), "rubble")):
        for idx in _rect(w, x0, y0, x1, y1):
            props.append((Cell(idx % w, idx // w), kind))
    victims = (*building.victims, Cell(33, 42), Cell(70, 42))
    blocked |= {c.y * w + c.x for c, _ in props} | {c.y * w + c.x for c in victims}
    grid = GridMap(w, h, resolution_m=0.5, blocked=frozenset(blocked),
                   origin_x_m=building.grid.origin_x_m, origin_y_m=building.grid.origin_y_m)
    sources = (*building.sources, HazardSource(Cell(58, 30), 0.9, "fire"), HazardSource(Cell(50, 12), 0.85, "fire"))
    initial = [0.0] * grid.size
    uncertainty = [0.12] * grid.size
    for src in sources:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                c = Cell(src.cell.x + dx, src.cell.y + dy)
                if grid.contains(c) and not grid.is_blocked(c):
                    i = grid.index(c)
                    initial[i] = max(initial[i], src.intensity * (0.42 if dx or dy else 1.0))
                    uncertainty[i] = min(1.0, uncertainty[i] + 0.24)
    waypoints = (*building.waypoints, Cell(60, 5), Cell(68, 40), Cell(33, 40))
    return Scenario(
        scenario_id="rescue_compound_v1", grid=grid,
        start=building.start, target=waypoints[0], base=building.base,
        refuges=(*building.refuges, Cell(68, 3), Cell(5, 40)),
        sources=sources, initial_risk=tuple(initial), initial_uncertainty=tuple(uncertainty),
        # 2 s layers x 120 = 240 s: with a 2-cell hazard standoff the longest leg plus its return can exceed 180 s
        # (seen on seed 1). Synthetic-model parameters as in rescue_building.
        forecast_config=ForecastConfig(horizon_steps=120, time_step_s=2.0, spread_rate=0.15, decay_rate=0.02,
                                       uncertainty_growth=0.018),
        waypoints=waypoints, victims=victims, props=tuple(props), min_passage_cells=3,
    )


SCENARIOS = {"corridor": corridor_scenario, "rescue_building": rescue_building_scenario,
             "rescue_compound": rescue_compound_scenario}


def scenario_by_name(name: str) -> Scenario:
    try:
        return SCENARIOS[name]()
    except KeyError:
        raise ValueError(f"unknown scenario {name!r}; known: {sorted(SCENARIOS)}") from None
