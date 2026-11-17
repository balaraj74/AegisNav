"""Dependency-free SVG renderer for comparable, reviewable route artifacts."""
from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Iterable, Sequence

from .models import Cell, GridMap, HazardFrame, RoutePlan


def render_map_svg(
    path: str | Path,
    grid: GridMap,
    frame: HazardFrame,
    start: Cell,
    target: Cell,
    route: RoutePlan | None = None,
    *,
    title: str = "AegisNav risk-aware route",
    other_route: RoutePlan | None = None,
    highlights: Iterable[Cell] = (),
) -> None:
    cell_px = 24
    margin = 36
    width_px = grid.width * cell_px + margin * 2
    height_px = grid.height * cell_px + margin * 2 + 42

    def point(cell: Cell) -> tuple[float, float]:
        return margin + (cell.x + 0.5) * cell_px, margin + (grid.height - cell.y - 0.5) * cell_px

    def path_points(plan: RoutePlan | None) -> str:
        if not plan or not plan.success:
            return ""
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in (point(c) for c in plan.cells))

    elements: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" viewBox="0 0 {width_px} {height_px}">',
        '<rect width="100%" height="100%" fill="#0b1220"/>',
        f'<text x="{margin}" y="24" font-family="sans-serif" font-size="16" font-weight="700" fill="#f8fafc">{escape(title)}</text>',
    ]
    highlight_set = set(highlights)
    for y in range(grid.height):
        for x in range(grid.width):
            cell = Cell(x, y)
            idx = grid.index(cell)
            val = frame.risk[idx]
            if idx in grid.blocked:
                fill = "#303846"
            elif val >= 0.80:
                fill = "#b91c1c"
            elif val >= 0.60:
                fill = "#ea580c"
            elif val >= 0.35:
                fill = "#f59e0b"
            elif val >= 0.15:
                fill = "#4b5563"
            else:
                fill = "#172235"
            rx = margin + x * cell_px
            ry = margin + (grid.height - 1 - y) * cell_px
            stroke = "#f8fafc" if cell in highlight_set else "#334155"
            stroke_width = "1.5" if cell in highlight_set else "0.5"
            elements.append(f'<rect x="{rx}" y="{ry}" width="{cell_px}" height="{cell_px}" fill="{fill}" stroke="{stroke}" stroke-width="{stroke_width}"/>')
    # The baseline is shown as dashed, risk-aware route as solid teal.
    other_points = path_points(other_route)
    if other_points:
        elements.append(f'<polyline points="{other_points}" fill="none" stroke="#fbbf24" stroke-width="3" stroke-dasharray="5 4" stroke-linejoin="round" stroke-linecap="round" opacity="0.95"/>')
    route_points = path_points(route)
    if route_points:
        elements.append(f'<polyline points="{route_points}" fill="none" stroke="#22d3ee" stroke-width="3.5" stroke-linejoin="round" stroke-linecap="round"/>')
    sx, sy = point(start)
    tx, ty = point(target)
    elements.extend([
        f'<circle cx="{sx}" cy="{sy}" r="7" fill="#22c55e" stroke="#f8fafc" stroke-width="1.5"/>',
        f'<text x="{sx + 9}" y="{sy - 7}" font-family="sans-serif" font-size="11" fill="#f8fafc">START / BASE</text>',
        f'<circle cx="{tx}" cy="{ty}" r="7" fill="#a78bfa" stroke="#f8fafc" stroke-width="1.5"/>',
        f'<text x="{tx + 9}" y="{ty - 7}" font-family="sans-serif" font-size="11" fill="#f8fafc">TARGET</text>',
        f'<text x="{margin}" y="{height_px - 18}" font-family="sans-serif" font-size="11" fill="#cbd5e1">Risk: green/gray = lower · amber = elevated · red = critical | cyan = AegisNav · dashed amber = baseline</text>',
        '</svg>',
    ])
    target_path = Path(path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    target_path.write_text("\n".join(elements), encoding="utf-8")
