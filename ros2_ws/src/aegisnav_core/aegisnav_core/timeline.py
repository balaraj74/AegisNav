"""Time-ordered event schedule derived from a DynamicScenario, for the ROS scenario node.

Pure and ROS-free: the node only decides *when* (its own clock) and publishes
what this module says is current. Times are seconds since mission start.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from .models import Cell, GridMap, HazardFrame, PlannerConfig
from .planning import RiskAwareAStar
from .simulation import DynamicScenario


@dataclass(frozen=True, slots=True)
class MissionSnapshot:
    """What the scenario says is true at `elapsed_s`."""

    confidence: float
    ignited_cells: tuple[tuple[Cell, float], ...]  # (cell, intensity), active so far


class ScenarioTimeline:
    def __init__(self, scenario: DynamicScenario) -> None:
        self._scenario = scenario

    def snapshot(self, elapsed_s: float) -> MissionSnapshot:
        if elapsed_s < 0:
            raise ValueError("elapsed_s must be non-negative")
        sc = self._scenario
        ignited = tuple((s.cell, s.intensity) for s in sc.ignition_sources) if elapsed_s >= sc.ignition_time_s else ()
        confidence = sc.confidence_after if elapsed_s >= sc.confidence_drop_time_s else sc.confidence_before
        return MissionSnapshot(confidence, ignited)

    @property
    def target(self) -> Cell:
        return self._scenario.base.target


def next_goal(current_goal: Cell, goals: Sequence[Cell], base: Cell, *, at_destination: bool, action: str) -> Cell:
    """Mission goal sequencing (victim waypoints in order, then base).

    The goal advances only when the robot is holding at the current goal under CONTINUE;
    refuge/return/abort decisions are destinations chosen by the policy, not by the goal.
    """
    if not (at_destination and action == "continue_to_target"):
        return current_goal
    if current_goal in goals:
        i = list(goals).index(current_goal)
        return goals[i + 1] if i + 1 < len(goals) else base
    return current_goal


@dataclass(frozen=True, slots=True)
class TriageResult:
    goal: Cell                       # next goal (base when nothing else is reachable)
    unreachable: tuple[Cell, ...]    # remaining goals with no safe route right now (retried later)


def triage_next_goal(current: Cell, remaining: Sequence[Cell], base: Cell, grid: GridMap,
                     frames: Sequence[HazardFrame], config: PlannerConfig, *, time_step_s: float) -> TriageResult:
    """Choose the remaining goal with a safe route of lowest (hazard exposure, distance) from `current`.

    Goals without a safe route are reported as unreachable rather than dropped; with none reachable
    the robot heads to base. Ties are broken by the order of `remaining`.
    """
    planner = RiskAwareAStar()
    candidates, unreachable = [], []
    for order, goal in enumerate(remaining):
        plan = planner.plan(grid, frames, current, goal, config, time_step_s=time_step_s)
        if plan.success:
            candidates.append((round(plan.hazard_exposure, 6), round(plan.distance_m, 6), order, goal))
        else:
            unreachable.append(goal)
    return TriageResult(min(candidates)[3] if candidates else base, tuple(unreachable))


class ScriptTimeline:
    """Fixed schedule of simulated events (named scenarios); same snapshot type as the seeded timeline."""

    def __init__(self, ignitions: Sequence[tuple[float, Cell, float]], confidence: Sequence[tuple[float, float]],
                 *, default_confidence: float = 0.95) -> None:
        for t, _cell, intensity in ignitions:
            if t < 0 or not 0.0 <= intensity <= 1.0:
                raise ValueError("ignition time must be >= 0 and intensity in [0, 1]")
        for t, value in confidence:
            if t < 0 or not 0.0 <= value <= 1.0:
                raise ValueError("confidence time must be >= 0 and value in [0, 1]")
        if not 0.0 <= default_confidence <= 1.0:
            raise ValueError("default_confidence must be in [0, 1]")
        self._ignitions = sorted(ignitions, key=lambda e: e[0])
        self._confidence = sorted(confidence)
        self._default = default_confidence

    def snapshot(self, elapsed_s: float) -> MissionSnapshot:
        if elapsed_s < 0:
            raise ValueError("elapsed_s must be non-negative")
        conf = self._default
        for t, value in self._confidence:
            if elapsed_s >= t:
                conf = value
        return MissionSnapshot(conf, tuple((c, i) for t, c, i in self._ignitions if elapsed_s >= t))
