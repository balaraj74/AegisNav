"""Mission-level policy: return feasibility, perception confidence and refuge selection."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .models import Cell, GridMap, HazardFrame, MissionAction, MissionAssessment, PlannerConfig
from .planning import RiskAwareAStar


@dataclass(frozen=True, slots=True)
class MissionPolicy:
    minimum_continue_confidence: float = 0.55
    critical_confidence: float = 0.30
    max_outbound_exposure: float = 18.0
    max_return_exposure: float = 18.0
    planner: PlannerConfig = PlannerConfig()

    def __post_init__(self) -> None:
        for name, value in (("minimum_continue_confidence", self.minimum_continue_confidence), ("critical_confidence", self.critical_confidence)):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.critical_confidence > self.minimum_continue_confidence:
            raise ValueError("critical confidence cannot exceed continue threshold")
        if self.max_outbound_exposure < 0 or self.max_return_exposure < 0:
            raise ValueError("Exposure budgets must be non-negative")


class ConfidenceHysteresis:
    """Latch degraded perception until confidence recovers to `resume` (> the continue threshold).

    Without it, a confidence hovering around `minimum_continue` flips the decision between continuing
    and holding every cycle. `update` returns the value the supervisor should use; callers keep
    reporting the measured value.
    """

    def __init__(self, minimum_continue: float, resume: float) -> None:
        if not 0.0 <= minimum_continue <= resume <= 1.0:
            raise ValueError("need 0 <= minimum_continue <= resume <= 1")
        self._minimum, self._resume, self._latched = minimum_continue, resume, False

    @property
    def latched(self) -> bool:
        return self._latched

    def update(self, confidence: float) -> float:
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if confidence < self._minimum:
            self._latched = True
        elif confidence >= self._resume:
            self._latched = False
        if self._latched:
            return min(confidence, math.nextafter(self._minimum, 0.0))
        return confidence


class MissionSafetySupervisor:
    """Pure decision policy; it does not command a real robot by itself."""

    def __init__(self, planner: RiskAwareAStar | None = None, policy: MissionPolicy = MissionPolicy()) -> None:
        self._planner = planner or RiskAwareAStar()
        self._policy = policy

    def assess(
        self,
        grid: GridMap,
        frames: Sequence[HazardFrame],
        current: Cell,
        target: Cell,
        base: Cell,
        refuges: Sequence[Cell],
        perception_confidence: float,
        *,
        time_step_s: float = 1.0,
    ) -> MissionAssessment:
        if not 0.0 <= perception_confidence <= 1.0:
            raise ValueError("perception_confidence must be in [0, 1]")
        if not frames:
            return MissionAssessment(
                MissionAction.ABORT, "missing_forecast", "No hazard forecast is available.",
                confidence=perception_confidence, forecast_horizon_sufficient=False,
            )
        horizon_steps = len(frames) - 1

        outbound = self._planner.plan(
            grid, frames, current, target, self._policy.planner, time_step_s=time_step_s,
        )
        direct_return = self._planner.plan(
            grid, frames, current, base, self._policy.planner, time_step_s=time_step_s,
        )
        direct_return_covered = direct_return.success and direct_return.duration_steps <= horizon_steps
        current_risk = frames[0].at(current)[0]

        def best_refuge(from_cell: Cell, start_step: int = 0):
            candidates = []
            for refuge in refuges:
                route = self._planner.plan(
                    grid, frames, from_cell, refuge, self._policy.planner,
                    start_time_step=start_step, time_step_s=time_step_s,
                )
                # Do not claim a route is forecast-feasible when its end exceeds
                # the forecast horizon. The planner's tail-clamp is for search
                # termination, not permission to claim future hazard knowledge.
                if route.success and start_step + route.duration_steps <= horizon_steps:
                    candidates.append((route.hazard_exposure, route.distance_m, refuge.x, refuge.y, refuge, route))
            return min(candidates, default=None)

        # Starting inside a critical cell is not the same as entering one. The
        # robot may need to escape it, so select an outbound safe route first.
        if current_risk >= self._policy.planner.hard_risk_threshold:
            refuge = best_refuge(current)
            if refuge:
                return MissionAssessment(
                    MissionAction.SEEK_REFUGE, "current_cell_critical",
                    "Current cell is above the hard risk threshold; leaving toward a covered low-exposure refuge.",
                    outbound=outbound, refuge_route=refuge[5], confidence=perception_confidence,
                    safe_return_feasible=False,
                )
            if direct_return_covered and direct_return.distance_m > 0.0:
                return MissionAssessment(
                    MissionAction.RETURN_TO_BASE, "current_cell_critical",
                    "Current cell is above the hard risk threshold; returning to base because no covered refuge route is available.",
                    outbound=outbound, return_route=direct_return, confidence=perception_confidence,
                    safe_return_feasible=True,
                )
            return MissionAssessment(
                MissionAction.ABORT, "current_cell_critical_no_exit",
                "Current cell is critically hazardous and no covered route away from it is available.",
                outbound=outbound, confidence=perception_confidence,
                safe_return_feasible=False, forecast_horizon_sufficient=False,
            )

        if perception_confidence <= self._policy.critical_confidence:
            refuge = best_refuge(current)
            if refuge:
                return MissionAssessment(
                    MissionAction.SEEK_REFUGE, "perception_confidence_critical",
                    "Perception confidence is critically low; moving to the lowest-exposure reachable refuge.",
                    outbound=outbound, refuge_route=refuge[5], confidence=perception_confidence,
                    safe_return_feasible=direct_return_covered,
                )
            if direct_return_covered:
                return MissionAssessment(
                    MissionAction.RETURN_TO_BASE, "perception_confidence_critical",
                    "Perception confidence is critically low; returning to base because no covered refuge route was found.",
                    outbound=outbound, return_route=direct_return, confidence=perception_confidence,
                    safe_return_feasible=True,
                )
            return MissionAssessment(
                MissionAction.ABORT, "no_safe_route_under_low_confidence",
                "Perception confidence is critically low and no fully forecast-covered base/refuge route was found.",
                outbound=outbound, confidence=perception_confidence, forecast_horizon_sufficient=False,
            )

        if perception_confidence < self._policy.minimum_continue_confidence:
            refuge = best_refuge(current)
            if refuge:
                return MissionAssessment(
                    MissionAction.SLOW_AND_REASSESS, "perception_confidence_degraded",
                    "Perception confidence is degraded; reassess from a reachable low-risk observation point before continuing.",
                    outbound=outbound, refuge_route=refuge[5], confidence=perception_confidence,
                    safe_return_feasible=direct_return_covered,
                )
            if direct_return_covered:
                return MissionAssessment(
                    MissionAction.RETURN_TO_BASE, "perception_confidence_degraded_no_observation_point",
                    "Perception confidence is degraded and no covered observation point is available; returning to base.",
                    outbound=outbound, return_route=direct_return, confidence=perception_confidence,
                    safe_return_feasible=True,
                )
            return MissionAssessment(
                MissionAction.ABORT, "degraded_perception_no_safe_route",
                "Perception confidence is degraded and no fully covered base/refuge route was found.",
                outbound=outbound, confidence=perception_confidence, forecast_horizon_sufficient=False,
            )

        if not outbound.success:
            refuge = best_refuge(current)
            if refuge:
                return MissionAssessment(
                    MissionAction.SEEK_REFUGE, "target_route_unavailable",
                    "No safe route to the target is available; selecting a low-exposure refuge.",
                    outbound=outbound, refuge_route=refuge[5], confidence=perception_confidence,
                    safe_return_feasible=direct_return_covered,
                )
            if direct_return_covered:
                return MissionAssessment(
                    MissionAction.RETURN_TO_BASE, "target_route_unavailable",
                    "No safe target/refuge route is available; returning to base.",
                    outbound=outbound, return_route=direct_return, confidence=perception_confidence,
                    safe_return_feasible=True,
                )
            return MissionAssessment(
                MissionAction.ABORT, "no_safe_route",
                "No fully forecast-covered safe route to target, base or refuge was found.",
                outbound=outbound, confidence=perception_confidence, forecast_horizon_sufficient=False,
            )

        if outbound.duration_steps > horizon_steps:
            refuge = best_refuge(current)
            if refuge:
                return MissionAssessment(
                    MissionAction.SEEK_REFUGE, "forecast_horizon_insufficient",
                    "The target route extends beyond the available forecast horizon; declining the mission and selecting a covered refuge route.",
                    outbound=outbound, refuge_route=refuge[5], confidence=perception_confidence,
                    safe_return_feasible=False, forecast_horizon_sufficient=False,
                )
            if direct_return_covered:
                return MissionAssessment(
                    MissionAction.RETURN_TO_BASE, "forecast_horizon_insufficient",
                    "The target route exceeds the forecast horizon; returning to base instead of planning beyond available hazard information.",
                    outbound=outbound, return_route=direct_return, confidence=perception_confidence,
                    safe_return_feasible=True, forecast_horizon_sufficient=False,
                )
            return MissionAssessment(
                MissionAction.ABORT, "forecast_horizon_insufficient",
                "The target route exceeds the forecast horizon and no fully covered refuge/base route is available.",
                outbound=outbound, confidence=perception_confidence,
                safe_return_feasible=False, forecast_horizon_sufficient=False,
            )

        return_route = self._planner.plan(
            grid, frames, target, base, self._policy.planner,
            start_time_step=outbound.duration_steps, time_step_s=time_step_s,
        )
        return_covered = return_route.success and outbound.duration_steps + return_route.duration_steps <= horizon_steps
        outbound_over_budget = outbound.hazard_exposure > self._policy.max_outbound_exposure
        return_over_budget = return_covered and return_route.hazard_exposure > self._policy.max_return_exposure

        if not return_covered or return_over_budget:
            refuge = best_refuge(current)
            if return_route.success and not return_covered:
                reason = "forecast_horizon_insufficient"
            elif not return_route.success:
                reason = "return_route_unavailable"
            else:
                reason = "return_exposure_over_budget"
            if refuge:
                return MissionAssessment(
                    MissionAction.SEEK_REFUGE, reason,
                    "The proposed target mission has no acceptable, fully forecast-covered return; selecting a covered low-exposure refuge route.",
                    outbound=outbound, return_route=return_route, refuge_route=refuge[5],
                    confidence=perception_confidence, safe_return_feasible=False,
                    forecast_horizon_sufficient=(reason != "forecast_horizon_insufficient"),
                )
            if direct_return_covered and direct_return.hazard_exposure <= self._policy.max_return_exposure:
                return MissionAssessment(
                    MissionAction.RETURN_TO_BASE, reason,
                    "The target mission cannot establish an acceptable return; return to base now.",
                    outbound=outbound, return_route=direct_return, confidence=perception_confidence,
                    safe_return_feasible=True, forecast_horizon_sufficient=(reason != "forecast_horizon_insufficient"),
                )
            return MissionAssessment(
                MissionAction.ABORT, reason,
                "No fully forecast-covered acceptable route from the proposed mission to base or refuge is available.",
                outbound=outbound, return_route=return_route, confidence=perception_confidence,
                safe_return_feasible=False, forecast_horizon_sufficient=(reason != "forecast_horizon_insufficient"),
            )

        if outbound_over_budget:
            if direct_return_covered and direct_return.hazard_exposure <= self._policy.max_return_exposure:
                return MissionAssessment(
                    MissionAction.RETURN_TO_BASE, "outbound_exposure_over_budget",
                    "Predicted exposure on the outbound path exceeds the configured budget; return to base.",
                    outbound=outbound, return_route=direct_return, confidence=perception_confidence,
                    safe_return_feasible=True,
                )
            refuge = best_refuge(current)
            if refuge:
                return MissionAssessment(
                    MissionAction.SEEK_REFUGE, "outbound_exposure_over_budget",
                    "The target route exceeds the exposure budget and direct return is unacceptable; move to a lower-risk refuge.",
                    outbound=outbound, refuge_route=refuge[5], confidence=perception_confidence,
                    safe_return_feasible=False,
                )
            return MissionAssessment(
                MissionAction.ABORT, "outbound_exposure_over_budget",
                "Predicted exposure exceeds the configured budget and no covered return/refuge route exists.",
                outbound=outbound, confidence=perception_confidence,
            )

        return MissionAssessment(
            MissionAction.CONTINUE_TO_TARGET, "mission_feasible",
            "The target route and forecast-time return route satisfy configured exposure, confidence and forecast-horizon constraints.",
            outbound=outbound, return_route=return_route, confidence=perception_confidence,
            safe_return_feasible=True, forecast_horizon_sufficient=True,
        )
