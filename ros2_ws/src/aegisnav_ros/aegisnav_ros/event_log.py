"""Bounded, de-duplicated event log for the on-screen overlay (pure, ROS-free)."""
from __future__ import annotations

import textwrap
from collections import deque


class EventLog:
    def __init__(self, *, max_lines: int = 7) -> None:
        if max_lines < 1:
            raise ValueError("max_lines must be at least 1")
        self._lines: deque[str] = deque(maxlen=max_lines)
        self._seen: set[str] = set()

    def add(self, t_s: float, text: str, *, key: str | None = None) -> bool:
        """Append "t=..s text"; an event with an already-seen key is ignored (returns False)."""
        if key is not None:
            if key in self._seen:
                return False
            self._seen.add(key)
        self._lines.append(f"t={t_s:5.1f}s  {text}")
        return True

    def lines(self) -> list[str]:
        return list(self._lines)


def wrap(text: str, width: int = 72, max_lines: int = 2) -> str:
    lines = textwrap.wrap(text, width) or [""]
    if len(lines) > max_lines:
        lines = lines[:max_lines]
        lines[-1] = lines[-1][: width - 3].rstrip() + "..."
    return "\n".join(lines)
