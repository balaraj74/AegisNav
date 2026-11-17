"""Render a run trace (simulation.TrialResult.trace) as a decision timeline SVG."""
from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Sequence

_ACTION_COLORS = {
    "continue_to_target": "#2f855a", "replan": "#b7791f", "slow_and_reassess": "#d69e2e",
    "return_to_base": "#2b6cb0", "seek_refuge": "#805ad5", "abort": "#c53030",
}
_MARKERS = {"hazard_ignition": ("#c05621", "ignition"), "confidence_change": ("#718096", "confidence"),
            "cancel": ("#c53030", "cancel"), "send": ("#2d3748", "send")}


def action_segments(trace: Sequence[dict], end_t: int) -> list[tuple[int, int, str, str]]:
    """(start, end, action, reason_code) where consecutive decisions with the same action+reason merge."""
    decisions = [e for e in trace if e["event"] == "decision"]
    segments: list[list] = []
    for event in decisions:
        key = (event["action"], event["reason_code"])
        if segments and (segments[-1][2], segments[-1][3]) == key:
            continue
        if segments:
            segments[-1][1] = event["t"]
        segments.append([event["t"], end_t, *key])
    return [tuple(s) for s in segments]


def render_timeline_svg(path: str | Path, trace: Sequence[dict], title: str) -> None:
    if not trace:
        raise ValueError("trace is empty")
    end_t = max(e["t"] for e in trace)
    end_t = max(end_t, 1)
    w, pad, bar_y, bar_h = 900, 40, 70, 34
    scale = (w - 2 * pad) / end_t
    x = lambda t: pad + t * scale
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="230" font-family="sans-serif" font-size="11">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="{pad}" y="24" font-size="14" font-weight="bold">{escape(title)}</text>',
           f'<text x="{pad}" y="{bar_y - 8}" fill="#555">decision (colour = action, label = reason_code) — simulated time, s</text>']
    for start, end, action, reason in action_segments(trace, end_t):
        width = max(2.0, (end - start) * scale)
        out.append(f'<rect x="{x(start):.1f}" y="{bar_y}" width="{width:.1f}" height="{bar_h}" fill="{_ACTION_COLORS.get(action, "#999")}"/>')
        out.append(f'<text x="{x(start) + 3:.1f}" y="{bar_y + 14}" fill="white">{escape(action)}</text>')
        out.append(f'<text x="{x(start) + 3:.1f}" y="{bar_y + 28}" fill="white" font-size="9">{escape(reason)}</text>')
    lane = {"hazard_ignition": 0, "confidence_change": 1, "cancel": 2, "send": 3}
    for event in trace:
        if event["event"] not in _MARKERS:
            continue
        color, label = _MARKERS[event["event"]]
        y = bar_y + bar_h + 22 + lane[event["event"]] * 16
        out.append(f'<circle cx="{x(event["t"]):.1f}" cy="{y}" r="4" fill="{color}"/>')
        out.append(f'<text x="{x(event["t"]) + 7:.1f}" y="{y + 4}" fill="{color}">{label} t={event["t"]}</text>')
    out.append(f'<line x1="{pad}" y1="{bar_y + bar_h}" x2="{w - pad}" y2="{bar_y + bar_h}" stroke="#444"/>')
    for t in range(0, end_t + 1, max(1, end_t // 10)):
        out.append(f'<text x="{x(t):.1f}" y="{bar_y + bar_h + 12}" text-anchor="middle" fill="#444" font-size="9">{t}</text>')
    out.append("</svg>")
    Path(path).write_text("\n".join(out), encoding="utf-8")
