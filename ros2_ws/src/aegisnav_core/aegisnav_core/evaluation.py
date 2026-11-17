"""Reproducible mission metrics; metrics are descriptive, not safety certification."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

from .models import RoutePlan


@dataclass(slots=True)
class TrialMetrics:
    scenario_id: str
    planner_name: str
    mission_success: bool
    safe_return_success: bool
    distance_m: float = 0.0
    estimated_time_s: float = 0.0
    hazard_exposure: float = 0.0
    uncertainty_exposure: float = 0.0
    replans: int = 0
    critical_risk_route: bool = False
    max_risk_encountered: float = 0.0
    notes: str = ""

    @classmethod
    def from_plan(cls, scenario_id: str, planner_name: str, plan: RoutePlan, *, safe_return_success: bool = False, replans: int = 0, risk_threshold: float = 0.92, notes: str = "") -> "TrialMetrics":
        return cls(
            scenario_id=scenario_id, planner_name=planner_name,
            mission_success=plan.success, safe_return_success=safe_return_success,
            distance_m=round(plan.distance_m, 4),
            estimated_time_s=round(plan.estimated_time_s, 4),
            hazard_exposure=round(plan.hazard_exposure, 4),
            uncertainty_exposure=round(plan.uncertainty_exposure, 4),
            replans=replans,
            critical_risk_route=bool(plan.success and plan.max_risk_encountered >= risk_threshold),
            max_risk_encountered=round(plan.max_risk_encountered, 4),
            notes=notes,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def write_metrics(path: str | Path, metrics: list[TrialMetrics]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps([m.to_dict() for m in metrics], indent=2) + "\n", encoding="utf-8")
