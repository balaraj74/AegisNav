"""Pure state machine that governs when a route may be sent to or cancelled on a controller.

It owns *only* the dispatch/cancel protocol (ADR-0003: a published path is not an
executed path). It has no ROS types: the ROS node translates `ExecCommand`s into
action-client calls and feeds goal responses/results back as events. The same
machine drives the closed-loop simulation, so the protocol is tested without ROS.

Invariants enforced here and covered by tests:
  * at most one goal is ever in flight; a replacement is sent only after the old
    goal has terminated (cancel acknowledged or result received);
  * a pending replacement is dropped if the policy later asks to stop;
  * repeated identical or suffix-equal routes do not cause cancel/resend churn;
  * repeated rejections/failures latch FAULT and stop dispatching until reset().
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Mapping

from .models import Cell, RoutePlan


class ExecState(StrEnum):
    IDLE = "idle"
    DISPATCHING = "dispatching"  # goal sent, acceptance not yet known
    EXECUTING = "executing"
    CANCELLING = "cancelling"  # cancel requested, termination not yet known
    FAULT = "fault"  # too many consecutive rejections/failures


class CommandKind(StrEnum):
    NONE = "none"
    SEND = "send"
    CANCEL = "cancel"


class GoalOutcome(StrEnum):
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    FAILED = "failed"  # controller aborted / goal rejected


@dataclass(frozen=True, slots=True)
class ExecCommand:
    kind: CommandKind
    cells: tuple[Cell, ...] = ()
    reason_code: str = "no_change"


_NONE = ExecCommand(CommandKind.NONE)


class ExecutionController:
    def __init__(self, *, max_consecutive_failures: int = 3, rejoin_cells: int = 2) -> None:
        if max_consecutive_failures < 1:
            raise ValueError("max_consecutive_failures must be at least 1")
        if rejoin_cells < 0:
            raise ValueError("rejoin_cells must be non-negative")
        self._rejoin = rejoin_cells
        self._max_failures = max_consecutive_failures
        self._state = ExecState.IDLE
        self._active: tuple[Cell, ...] = ()
        self._pending: tuple[Cell, ...] | None = None
        self._failures = 0
        self.sent_count = 0
        self.cancel_count = 0

    @property
    def state(self) -> ExecState:
        return self._state

    @property
    def active_cells(self) -> tuple[Cell, ...]:
        return self._active

    @property
    def pending_cells(self) -> tuple[Cell, ...] | None:
        return self._pending

    def reset(self) -> None:
        """Operator/supervisor acknowledgement of a FAULT; clears the failure latch."""
        if self._state is ExecState.FAULT:
            self._state = ExecState.IDLE
        self._failures = 0
        self._pending = None

    def desired(self, cells: tuple[Cell, ...] | None) -> ExecCommand:
        """Declare the route the policy wants executed now (None = stop / do not move)."""
        if cells is not None and len(cells) == 0:
            raise ValueError("A desired route must contain at least one cell")
        if self._state is ExecState.FAULT:
            return ExecCommand(CommandKind.NONE, reason_code="execution_fault_latched")

        if cells is None:
            self._pending = None
            if self._state in (ExecState.DISPATCHING, ExecState.EXECUTING):
                return self._begin_cancel("no_safe_route_cancel")
            return _NONE

        if self._state is ExecState.IDLE:
            return self._begin_send(cells, "dispatch")
        if self._state is ExecState.CANCELLING:
            self._pending = cells
            return ExecCommand(CommandKind.NONE, reason_code="awaiting_cancel_ack")
        # DISPATCHING or EXECUTING
        if self._is_same_route(cells):
            return _NONE
        self._pending = cells
        return self._begin_cancel("replan_cancel")

    def on_goal_response(self, accepted: bool) -> ExecCommand:
        if self._state is ExecState.CANCELLING:
            # Cancel was requested before the server answered. A late-accepted
            # goal must be cancelled explicitly; a rejected one never ran.
            if accepted:
                return ExecCommand(CommandKind.CANCEL, reason_code="cancel_late_accepted_goal")
            return self.on_result(GoalOutcome.CANCELED)
        if self._state is not ExecState.DISPATCHING:
            raise RuntimeError(f"goal response received in state {self._state.value}")
        if accepted:
            self._state = ExecState.EXECUTING
            return _NONE
        return self._on_failure("goal_rejected")

    def on_result(self, outcome: GoalOutcome) -> ExecCommand:
        """Terminal event for the in-flight goal. Returns a SEND if a replacement is pending."""
        if self._state not in (ExecState.DISPATCHING, ExecState.EXECUTING, ExecState.CANCELLING):
            raise RuntimeError(f"goal result received in state {self._state.value}")
        if outcome is GoalOutcome.FAILED:
            return self._on_failure("controller_failed")
        if outcome is GoalOutcome.SUCCEEDED:
            self._failures = 0
        self._state = ExecState.IDLE
        self._active = ()
        pending, self._pending = self._pending, None
        if pending is not None:
            return self._begin_send(pending, "replacement_dispatch")
        return _NONE

    def _is_same_route(self, cells: tuple[Cell, ...]) -> bool:
        # A replan issued mid-route starts at the robot's current cell. The route
        # is unchanged if, after at most `rejoin_cells` leading cells (the robot
        # is slightly off the old path under a real controller), it equals a
        # suffix of the active route. The goal cell must match in every case.
        for skip in range(0, self._rejoin + 1):
            tail = cells[skip:]
            if tail and len(tail) <= len(self._active) and self._active[len(self._active) - len(tail):] == tail:
                return True
        return False

    def _begin_send(self, cells: tuple[Cell, ...], reason: str) -> ExecCommand:
        self._state = ExecState.DISPATCHING
        self._active = cells
        self.sent_count += 1
        return ExecCommand(CommandKind.SEND, cells, reason)

    def _begin_cancel(self, reason: str) -> ExecCommand:
        self._state = ExecState.CANCELLING
        self.cancel_count += 1
        return ExecCommand(CommandKind.CANCEL, reason_code=reason)

    def _on_failure(self, reason: str) -> ExecCommand:
        self._failures += 1
        self._active = ()
        self._pending = None
        self._state = ExecState.FAULT if self._failures >= self._max_failures else ExecState.IDLE
        return ExecCommand(CommandKind.NONE, reason_code=reason)


def stale_inputs(now_s: float, stamps_s: Mapping[str, float | None], max_age_s: float) -> list[str]:
    """Names of inputs that are missing or older than max_age_s (same time base as now_s).

    A planning cycle must not run while this list is non-empty; the caller
    publishes a fail-closed decision instead.
    """
    if max_age_s <= 0:
        raise ValueError("max_age_s must be positive")
    return sorted(
        name for name, stamp in stamps_s.items()
        if stamp is None or stamp > now_s or now_s - stamp > max_age_s
    )


def remaining_route(active: tuple[Cell, ...], current: Cell) -> tuple[Cell, ...] | None:
    """Part of the active route still ahead of the robot, or None if the robot has left it.

    A robot one cell (8-neighbourhood) off the route, as a real controller tracks it,
    resumes at the furthest cell of the first adjacent run of route cells.
    """
    if current in active:
        return active[active.index(current):]
    adjacent = [i for i, c in enumerate(active) if max(abs(c.x - current.x), abs(c.y - current.y)) <= 1]
    if not adjacent:
        return None
    end = adjacent[0]
    while end + 1 in adjacent:
        end += 1
    return active[end:]


def keep_active_route(active_remaining: RoutePlan, candidate: RoutePlan, *, min_relative_improvement: float) -> bool:
    """Hysteresis: keep the active route unless the candidate is materially better.

    The active route is replaced when it is no longer acceptable (evaluation
    failed), when the destination changed, or when the candidate costs at least
    `min_relative_improvement` (fraction in [0, 1)) less under the same forecast.
    """
    if not 0.0 <= min_relative_improvement < 1.0:
        raise ValueError("min_relative_improvement must be in [0, 1)")
    if not active_remaining.success or not candidate.success:
        return False
    if active_remaining.cells[-1] != candidate.cells[-1]:
        return False
    return candidate.total_cost > active_remaining.total_cost * (1.0 - min_relative_improvement)
