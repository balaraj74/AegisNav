"""Pure shot selection for the recording camera (ROS-free).

The director never influences the robot; it only picks what the Gazebo GUI camera shows:
an establishing overview at the start, a chase camera by default, and short cut-aways to a new
fire or to a top-down tactical view when the decision changes.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Shot:
    kind: str                       # "overview" | "chase" | "fire" | "tactical"
    target_xy: tuple[float, float] = (0.0, 0.0)


class Director:
    def __init__(self, *, overview_s: float = 6.0, cutaway_s: float = 5.0) -> None:
        if overview_s < 0 or cutaway_s <= 0:
            raise ValueError("overview_s must be >= 0 and cutaway_s > 0")
        self._overview_s, self._cutaway_s = overview_s, cutaway_s
        self._cut: tuple[float, Shot] | None = None  # (until_t, shot)

    def on_fire(self, t: float, x: float, y: float) -> None:
        # Ignitions during a running fire cut-away (e.g. several cells at once) keep the current shot:
        # restarting it re-aims the camera mid-move.
        if self._cut is not None and self._cut[1].kind == "fire" and t < self._cut[0]:
            return
        self._cut = (t + self._cutaway_s, Shot("fire", (x, y)))

    def on_decision_change(self, t: float) -> None:
        # A fire cut-away in progress is not interrupted: it explains the decision that follows.
        if self._cut is None or self._cut[1].kind != "fire" or t >= self._cut[0]:
            self._cut = (t + self._cutaway_s, Shot("tactical"))

    def shot(self, t: float) -> Shot:
        if t < self._overview_s:
            return Shot("overview")
        if self._cut is not None and t < self._cut[0]:
            return self._cut[1]
        return Shot("chase")
