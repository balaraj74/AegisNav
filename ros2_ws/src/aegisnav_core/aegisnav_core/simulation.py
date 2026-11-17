"""Deterministic closed-loop mission simulation (no ROS, no physics engine).

This is a *kinematic abstraction* used to exercise the decision loop end to end:
hazard truth -> belief forecast -> mission policy -> execution state machine ->
simulated action server -> robot motion -> measured exposure. It is not a
substitute for the Gazebo/Nav2 integration, which has not been run on this host.

Time base: integer simulation seconds, dt = 1 s. Robot motion is a point robot
that advances along the dispatched route at `robot_speed_mps` and only while the
action server reports EXECUTING. The robot is treated as occupying the last grid
cell it has reached. Hazard "truth" is the synthetic diffusion field in
`forecast.py` with an optional ignition event; the robot's belief at time t is
the same model re-initialised from truth(t) and only the sources it knows about.
"""
from __future__ import annotations

import math
import random
import time
from bisect import bisect_right
from dataclasses import dataclass, field, fields, replace
from enum import StrEnum

from .execution import (
    CommandKind, ExecCommand, ExecState, ExecutionController, GoalOutcome, keep_active_route, remaining_route,
)
from .forecast import ForecastResult, HazardForecaster
from .mission import MissionPolicy, MissionSafetySupervisor
from .models import Cell, ForecastConfig, HazardFrame, HazardSource, MissionAction, PlannerConfig, RoutePlan
from .planning import RiskAwareAStar, evaluate_route
from .scenarios import Scenario, corridor_scenario
from .sensing import HazardBelief, observe, visible_cells


class Outcome(StrEnum):
    COMPLETED_AND_RETURNED = "completed_and_returned"
    RETURNED_EARLY = "returned_early"
    REACHED_REFUGE = "reached_refuge"
    ABORTED = "aborted"
    PLAN_FAILED = "plan_failed"
    TIMEOUT = "timeout"


class Mode(StrEnum):
    AEGISNAV = "aegisnav_risk_aware"
    AEGISNAV_SENSING = "aegisnav_onboard_sensing"  # knows only hazards its simulated sensor has seen
    BASELINE = "distance_only_baseline"


@dataclass(frozen=True, slots=True)
class DynamicScenario:
    """Everything that varies between trials; all times are simulation seconds."""

    seed: int
    base: Scenario
    ignition_time_s: int
    ignition_sources: tuple[HazardSource, ...]
    confidence_drop_time_s: int
    confidence_before: float
    confidence_after: float
    robot_speed_mps: float
    max_time_s: int = 240
    reassess_period_s: int = 5
    near_risk: float = 0.5

    def __post_init__(self) -> None:
        if self.ignition_time_s < 1 or self.confidence_drop_time_s < 1:
            raise ValueError("event times must be at least 1 s")
        for name, value in (("confidence_before", self.confidence_before), ("confidence_after", self.confidence_after), ("near_risk", self.near_risk)):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.robot_speed_mps <= 0 or self.max_time_s < 1 or self.reassess_period_s < 1:
            raise ValueError("robot_speed_mps, max_time_s and reassess_period_s must be positive")
        if self.base.forecast_config.time_step_s != 1.0:
            raise ValueError("the closed-loop runner requires a 1 s forecast time step")
        for source in self.ignition_sources:
            if not self.base.grid.contains(source.cell) or self.base.grid.is_blocked(source.cell):
                raise ValueError(f"ignition source {source.cell} is outside the map or in a blocked cell")

    def confidence_at(self, t: int) -> float:
        return self.confidence_after if t >= self.confidence_drop_time_s else self.confidence_before


def seeded_dynamic_scenario(seed: int) -> DynamicScenario:
    """Reference scenario family; all randomness comes from `seed`.

    Per seed: the ignition cell is one of four cells in or beside the upper gate (on
    the initial AegisNav route) or one of two cells away from both routes; the
    ignition time and intensity, the robot speed and the confidence-drop time and
    level are drawn from fixed ranges; confidence degrades in about half of the
    trials. These distributions are a design choice of this study, not field data.
    """
    rng = random.Random(seed)
    base = corridor_scenario()
    # Four candidates threaten the upper gate (cells (11, 11)-(11, 12)): two inside it and
    # two directly beside it. Two are away from both routes.
    candidates = (Cell(11, 11), Cell(11, 12), Cell(10, 12), Cell(12, 11), Cell(20, 4), Cell(4, 10))
    cell = candidates[rng.randrange(len(candidates))]
    ignition_time = rng.randint(12, 30)
    intensity = round(rng.uniform(0.93, 1.0), 3)
    drop_time = rng.randint(25, 60)
    degrades = rng.random() < 0.5
    confidence_after = round(rng.uniform(0.15, 0.5), 3) if degrades else 0.95
    return DynamicScenario(
        seed=seed, base=base,
        ignition_time_s=ignition_time,
        ignition_sources=(HazardSource(cell, intensity, "ignition"),),
        confidence_drop_time_s=drop_time,
        confidence_before=0.95,
        confidence_after=confidence_after,
        robot_speed_mps=round(rng.uniform(0.30, 0.40), 3),
    )


class TruthField:
    """Ground-truth hazard for t = 0..max_time_s; the tail is held beyond that."""

    def __init__(self, ds: DynamicScenario) -> None:
        base, cfg = ds.base, ds.base.forecast_config
        forecaster = HazardForecaster()
        full = replace(cfg, horizon_steps=ds.max_time_s)
        pre = forecaster.forecast(base.grid, base.initial_risk, base.initial_uncertainty, base.sources, full)
        k = ds.ignition_time_s
        post = forecaster.forecast(
            base.grid, pre.frames[k].risk, pre.frames[k].uncertainty,
            base.sources + ds.ignition_sources, replace(cfg, horizon_steps=ds.max_time_s - k + 1),
        )
        frames: list[HazardFrame] = []
        for t in range(ds.max_time_s + 1):
            src = pre.frames[t] if t < k else post.frames[t - k]
            frames.append(HazardFrame(src.width, src.height, src.risk, src.uncertainty, t, float(t)))
        self.frames = tuple(frames)

    def frame(self, t: int) -> HazardFrame:
        return self.frames[min(max(t, 0), len(self.frames) - 1)]

    def risk_at(self, t: int, cell: Cell) -> float:
        return self.frame(t).at(cell)[0]


def belief_forecast(ds: DynamicScenario, truth: TruthField, t: int) -> ForecastResult:
    """Forecast from truth(t) forward using only sources known at time t."""
    sources = ds.base.sources + (ds.ignition_sources if t >= ds.ignition_time_s else ())
    now = truth.frame(t)
    return HazardForecaster().forecast(
        ds.base.grid, now.risk, now.uncertainty, sources, ds.base.forecast_config,
    )


@dataclass(slots=True)
class TrialResult:
    seed: int
    mode: str
    outcome: Outcome
    reached_target: bool
    returned_to_base: bool
    safe_return: bool
    safe_outcome: bool
    hazard_exposure: float
    time_near_hazard_s: int
    time_in_hazard_s: int
    max_risk_encountered: float
    replans: int
    cancels: int
    path_length_m: float
    mission_time_s: int
    planning_time_ms_mean: float
    planning_time_ms_max: float
    trace: list[dict] = field(default_factory=list)
    trajectory: list[Cell] = field(default_factory=list)

    def metrics_dict(self) -> dict:
        out = {}
        for f in fields(self):
            if f.name in ("trace", "trajectory"):
                continue
            value = getattr(self, f.name)
            out[f.name] = value.value if isinstance(value, Outcome) else value
        return out

    def deterministic_signature(self) -> dict:
        """Everything except wall-clock planning time; equal across replays of one seed."""
        data = self.metrics_dict()
        data.pop("planning_time_ms_mean"), data.pop("planning_time_ms_max")
        data["trace"] = self.trace
        data["trajectory"] = [(c.x, c.y) for c in self.trajectory]
        return data


_BASELINE_PLANNER = PlannerConfig(hazard_weight=0.0, uncertainty_weight=0.0, hard_risk_threshold=0.99)


class _SimulatedActionServer:
    """Stand-in for a FollowPath server: one tick to accept, one tick to confirm a cancel."""

    def __init__(self) -> None:
        self.cells: tuple[Cell, ...] = ()
        self.cum: list[float] = []
        self.progress = 0.0

    def load(self, cells: tuple[Cell, ...], resolution_m: float) -> None:
        self.cells, self.progress, self.cum = cells, 0.0, [0.0]
        for a, b in zip(cells, cells[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b.x - a.x, b.y - a.y) * resolution_m)

    def advance(self, distance_m: float) -> tuple[Cell, float, bool]:
        step = min(distance_m, self.cum[-1] - self.progress)
        self.progress += step
        idx = bisect_right(self.cum, self.progress + 1e-9) - 1
        return self.cells[idx], step, self.progress >= self.cum[-1] - 1e-9


def run_trial(
    ds: DynamicScenario, truth: TruthField, mode: Mode, policy: MissionPolicy | None = None,
    *, replan_min_improvement: float = 0.15, sensor_range_m: float = 4.0,
) -> TrialResult:
    grid, base_cell, target = ds.base.grid, ds.base.base, ds.base.target
    policy = policy or MissionPolicy()
    hard = policy.planner.hard_risk_threshold
    planner = RiskAwareAStar()
    supervisor = MissionSafetySupervisor(planner, policy)
    ctl = ExecutionController()
    server = _SimulatedActionServer()

    cell = ds.base.start
    phase = "outbound"
    outcome: Outcome | None = None
    trace: list[dict] = []
    trajectory = [cell]
    planning_ms: list[float] = []
    exposure = 0.0
    near_s = in_s = 0
    max_risk = truth.risk_at(0, cell)
    path_m = 0.0
    replans = 0
    last_conf = ds.confidence_at(0)
    sensed = HazardBelief(grid) if mode is Mode.AEGISNAV_SENSING else None

    def log(t: int, event: str, **data) -> None:
        trace.append({"t": t, "event": event, **data})

    def apply(t: int, cmd: ExecCommand) -> None:
        nonlocal replans
        if cmd.kind is CommandKind.SEND:
            server.load(cmd.cells, grid.resolution_m)
            replans += cmd.reason_code == "replacement_dispatch"
            log(t, "send", reason_code=cmd.reason_code, waypoints=len(cmd.cells),
                goal=[cmd.cells[-1].x, cmd.cells[-1].y])
        elif cmd.kind is CommandKind.CANCEL:
            log(t, "cancel", reason_code=cmd.reason_code)

    def decide(t: int, belief_frames: tuple[HazardFrame, ...]) -> tuple[RoutePlan | None, MissionAction, str]:
        """Return (desired route or None, action, reason_code) for this tick."""
        goal = target if phase == "outbound" else base_cell
        started = time.perf_counter()
        if mode is not Mode.BASELINE:
            conf = ds.confidence_at(t)
            assessment = supervisor.assess(
                grid, belief_frames, cell, goal, base_cell, ds.base.refuges, conf,
                time_step_s=ds.base.forecast_config.time_step_s,
            )
            planning_ms.append((time.perf_counter() - started) * 1e3)
            route = assessment.execution_route
            log(t, "decision", action=assessment.action.value, reason_code=assessment.reason_code,
                confidence=conf, explanation=assessment.explanation,
                route_waypoints=len(route.cells) if route else 0)
            return route, assessment.action, assessment.reason_code
        plan = planner.plan(grid, belief_frames, cell, goal, _BASELINE_PLANNER,
                            time_step_s=ds.base.forecast_config.time_step_s)
        planning_ms.append((time.perf_counter() - started) * 1e3)
        action = MissionAction.CONTINUE_TO_TARGET if plan.success else MissionAction.ABORT
        log(t, "decision", action=action.value, reason_code="distance_only" if plan.success else plan.reason,
            route_waypoints=len(plan.cells))
        return (plan if plan.success else None), action, "distance_only" if plan.success else plan.reason

    for t in range(ds.max_time_s + 1):
        if t == ds.ignition_time_s:
            log(t, "hazard_ignition", sources=[[s.cell.x, s.cell.y] for s in ds.ignition_sources])
        conf_now = ds.confidence_at(t)
        if conf_now != last_conf:
            log(t, "confidence_change", confidence=conf_now)
            last_conf = conf_now

        if phase == "outbound" and cell == target and ctl.state is ExecState.IDLE:
            phase = "return"
            log(t, "phase", phase="return")
        if phase == "return" and cell == base_cell and ctl.state is ExecState.IDLE:
            outcome = Outcome.COMPLETED_AND_RETURNED
            break

        event_tick = t in (ds.ignition_time_s, ds.confidence_drop_time_s)
        if sensed is not None:
            visible = visible_cells(grid, cell, sensor_range_m)
            discovered = sensed.update(observe(truth.frame(t).risk, (i for i in visible if i not in grid.blocked)), t)
            if discovered:
                log(t, "hazard_discovered", cells=[[c.x, c.y] for c in discovered])
                event_tick = True
            # The sensing robot cannot react to an ignition it has not seen.
            event_tick = event_tick and (bool(discovered) or t == ds.confidence_drop_time_s)
        due = (t % ds.reassess_period_s == 0) or event_tick or ctl.state is ExecState.IDLE
        if mode is Mode.BASELINE:
            due = ctl.state is ExecState.IDLE
        if due and ctl.state is not ExecState.FAULT:
            if sensed is not None:
                risk0, uncertainty0 = sensed.layers(t)
                belief = HazardForecaster().forecast(grid, risk0, uncertainty0, sensed.sources(), ds.base.forecast_config)
            else:
                belief = belief_forecast(ds, truth, t)
            route, action, reason = decide(t, belief.frames)
            desired = route.cells if route else None
            if route is not None and ctl.state is ExecState.EXECUTING:
                ahead = remaining_route(ctl.active_cells, cell)
                if ahead is not None:
                    active_eval = evaluate_route(grid, belief.frames, ahead, policy.planner,
                                                 time_step_s=ds.base.forecast_config.time_step_s)
                    if keep_active_route(active_eval, route, min_relative_improvement=replan_min_improvement):
                        desired = ahead  # suffix of the active route: no cancel/resend
            at_destination = desired is not None and len(desired) == 1 and desired[0] == cell
            if action is not MissionAction.CONTINUE_TO_TARGET and at_destination and ctl.state is ExecState.IDLE:
                outcome = {
                    MissionAction.RETURN_TO_BASE: Outcome.RETURNED_EARLY,
                    MissionAction.SEEK_REFUGE: Outcome.REACHED_REFUGE,
                    MissionAction.SLOW_AND_REASSESS: Outcome.REACHED_REFUGE,
                }[action]
                log(t, "terminal", outcome=outcome.value, reason_code=reason)
                break
            if action is MissionAction.ABORT and desired is None:
                apply(t, ctl.desired(None))
                if ctl.state is ExecState.IDLE:
                    outcome = Outcome.ABORTED if mode is not Mode.BASELINE else Outcome.PLAN_FAILED
                    log(t, "terminal", outcome=outcome.value, reason_code=reason)
                    break
            elif desired is not None and not at_destination:
                apply(t, ctl.desired(desired))

        # Simulated action server: acknowledge, confirm cancel, or move.
        if ctl.state is ExecState.DISPATCHING:
            log(t, "goal_response", accepted=True)
            ctl.on_goal_response(True)
        elif ctl.state is ExecState.CANCELLING:
            log(t, "result", outcome=GoalOutcome.CANCELED.value)
            apply(t, ctl.on_result(GoalOutcome.CANCELED))
        elif ctl.state is ExecState.EXECUTING:
            cell, step, finished = server.advance(ds.robot_speed_mps)
            path_m += step
            if finished:
                log(t, "result", outcome=GoalOutcome.SUCCEEDED.value)
                ctl.on_result(GoalOutcome.SUCCEEDED)

        trajectory.append(cell)
        risk = truth.risk_at(t, cell)
        exposure += risk
        near_s += risk >= ds.near_risk
        in_s += risk >= hard
        max_risk = max(max_risk, risk)
    else:
        outcome = Outcome.TIMEOUT
        log(ds.max_time_s, "terminal", outcome=outcome.value, reason_code="max_time_reached")

    reached_target = any(e["event"] == "phase" for e in trace) or cell == target
    returned = outcome in (Outcome.COMPLETED_AND_RETURNED, Outcome.RETURNED_EARLY) and cell == base_cell
    return TrialResult(
        seed=ds.seed, mode=mode.value, outcome=outcome, reached_target=reached_target,
        returned_to_base=returned,
        safe_return=bool(outcome is Outcome.COMPLETED_AND_RETURNED and max_risk < hard),
        safe_outcome=bool(outcome in (Outcome.COMPLETED_AND_RETURNED, Outcome.RETURNED_EARLY, Outcome.REACHED_REFUGE) and max_risk < hard),
        hazard_exposure=round(exposure, 4), time_near_hazard_s=near_s, time_in_hazard_s=in_s,
        max_risk_encountered=round(max_risk, 4), replans=replans, cancels=ctl.cancel_count,
        path_length_m=round(path_m, 4), mission_time_s=len(trajectory) - 1,
        planning_time_ms_mean=round(sum(planning_ms) / len(planning_ms), 3) if planning_ms else 0.0,
        planning_time_ms_max=round(max(planning_ms), 3) if planning_ms else 0.0,
        trace=trace, trajectory=trajectory,
    )
