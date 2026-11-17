"""Pure helpers for the operator console: live mission metrics and input validation (ROS-free)."""
from __future__ import annotations

import math


class MissionMetrics:
    """Integrates what the robot actually experienced, sampled from the *current* forecast layer.

    exposure_risk_s: sum of risk x dt at the robot's cell; near/in: seconds at risk >= near/hard.
    These are simulated-model measures, not physical dose.
    """

    def __init__(self, *, near_risk: float = 0.5, hard_risk: float = 0.92) -> None:
        if not 0.0 < near_risk <= hard_risk <= 1.0:
            raise ValueError("need 0 < near_risk <= hard_risk <= 1")
        self.near_risk, self.hard_risk = near_risk, hard_risk
        self.distance_m = self.exposure_risk_s = self.time_near_s = self.time_in_s = 0.0
        self.peak_risk = 0.0
        self._last: tuple[float, float, float] | None = None

    def update(self, t_s: float, x: float, y: float, risk_here: float) -> None:
        if not all(math.isfinite(v) for v in (t_s, x, y, risk_here)):
            raise ValueError("non-finite metric sample")
        if self._last is not None:
            t0, x0, y0 = self._last
            dt = t_s - t0
            if dt <= 0:
                return  # out-of-order or repeated sample
            self.distance_m += math.hypot(x - x0, y - y0)
            self.exposure_risk_s += risk_here * dt
            self.time_near_s += dt if risk_here >= self.near_risk else 0.0
            self.time_in_s += dt if risk_here >= self.hard_risk else 0.0
        self.peak_risk = max(self.peak_risk, risk_here)
        self._last = (t_s, x, y)

    def as_dict(self) -> dict:
        return {"distance_m": round(self.distance_m, 2), "exposure_risk_s": round(self.exposure_risk_s, 2),
                "time_near_s": round(self.time_near_s, 1), "time_in_s": round(self.time_in_s, 1),
                "peak_risk": round(self.peak_risk, 3)}


def parse_point(payload: dict) -> tuple[float, float]:
    x, y = float(payload["x"]), float(payload["y"])
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("x and y must be finite")
    return x, y


def parse_confidence(payload: dict) -> float:
    v = float(payload["value"])
    if not 0.0 <= v <= 1.0:
        raise ValueError("confidence must be in [0, 1]")
    return v
