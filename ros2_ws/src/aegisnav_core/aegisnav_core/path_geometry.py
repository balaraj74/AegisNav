"""Convert a grid route (cell centres) into metric waypoints for a path-following controller.

Grid routes pass through cell centres, so in a two-cell-wide opening they hug one wall.
For the straight (axis-aligned) step through a cell, a waypoint with an obstacle on one
side and free space on the other is shifted half a cell toward the free side. The shifted point lies on
the boundary between two free cells, so it never enters an obstacle; cost/risk semantics
of the route are unchanged (the robot body overlaps both cells either way).
"""
from __future__ import annotations

from typing import Sequence

from .models import Cell, GridMap


def _free(grid: GridMap, cell: Cell) -> bool:
    return grid.contains(cell) and not grid.is_blocked(cell)


def centred_waypoints(grid: GridMap, cells: Sequence[Cell]) -> list[tuple[float, float]]:
    points = [grid.cell_center_world(c) for c in cells]
    for i in range(1, len(cells) - 1):
        prev, cur, nxt = cells[i - 1], cells[i], cells[i + 1]
        # Use the straight (axis-aligned) step through this cell: the pass itself if it is
        # straight, otherwise the outgoing step, otherwise the incoming one (e.g. a diagonal
        # entry into a gate followed by a straight exit is centred on the exit).
        for dx, dy in ((nxt.x - prev.x, nxt.y - prev.y), (nxt.x - cur.x, nxt.y - cur.y), (cur.x - prev.x, cur.y - prev.y)):
            if (dx == 0) != (dy == 0):
                break
        else:
            continue
        px, py = (-1 if dy > 0 else 1, 0) if dx == 0 else (0, 1 if dx > 0 else -1)
        side_a, side_b = Cell(cur.x + px, cur.y + py), Cell(cur.x - px, cur.y - py)
        shift = 0
        if not _free(grid, side_a) and _free(grid, side_b):
            shift = -1
        elif not _free(grid, side_b) and _free(grid, side_a):
            shift = 1
        if shift:
            x, y = points[i]
            points[i] = (x + shift * px * grid.resolution_m / 2, y + shift * py * grid.resolution_m / 2)
    return points


def densify(points: Sequence[tuple[float, float]], max_step_m: float) -> list[tuple[float, float]]:
    """Insert evenly spaced points so consecutive points are at most max_step_m apart.

    Pure-pursuit controllers pick a lookahead point along the path; sparse 0.4 m waypoints
    make that point jump and the robot cut corners near walls.
    """
    if max_step_m <= 0:
        raise ValueError("max_step_m must be positive")
    if not points:
        return []
    out = [points[0]]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        n = max(1, int(-(-((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5 // max_step_m)))
        out.extend((x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(1, n + 1))
    return out
