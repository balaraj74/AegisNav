import unittest

from aegisnav_core.execution import (
    CommandKind, ExecState, ExecutionController, GoalOutcome, stale_inputs,
)
from aegisnav_core.models import Cell

A = (Cell(0, 0), Cell(1, 0), Cell(2, 0))
B = (Cell(0, 0), Cell(0, 1), Cell(0, 2))


def running(ctl: ExecutionController, route=A) -> None:
    assert ctl.desired(route).kind is CommandKind.SEND
    ctl.on_goal_response(True)
    assert ctl.state is ExecState.EXECUTING


class ExecutionControllerTests(unittest.TestCase):
    def test_idle_to_send_to_executing_to_success(self):
        ctl = ExecutionController()
        cmd = ctl.desired(A)
        self.assertEqual((cmd.kind, cmd.cells, ctl.state), (CommandKind.SEND, A, ExecState.DISPATCHING))
        ctl.on_goal_response(True)
        self.assertEqual(ctl.state, ExecState.EXECUTING)
        self.assertEqual(ctl.on_result(GoalOutcome.SUCCEEDED).kind, CommandKind.NONE)
        self.assertEqual(ctl.state, ExecState.IDLE)

    def test_identical_and_suffix_routes_do_not_churn(self):
        ctl = ExecutionController()
        running(ctl)
        self.assertEqual(ctl.desired(A).kind, CommandKind.NONE)
        self.assertEqual(ctl.desired(A[1:]).kind, CommandKind.NONE)  # replan from a later pose
        self.assertEqual((ctl.sent_count, ctl.cancel_count), (1, 0))

    def test_route_rejoining_active_path_is_not_a_replan(self):
        ctl = ExecutionController()
        long = tuple(Cell(x, 0) for x in range(6))
        running(ctl, long)
        off_path = (Cell(2, 1), Cell(3, 0), Cell(4, 0), Cell(5, 0))  # robot drifted one cell sideways
        self.assertEqual(ctl.desired(off_path).kind, CommandKind.NONE)

    def test_rejoin_tolerance_is_bounded_and_goal_must_match(self):
        ctl = ExecutionController(rejoin_cells=1)
        long = tuple(Cell(x, 0) for x in range(6))
        running(ctl, long)
        three_off = (Cell(2, 2), Cell(2, 1), Cell(3, 1), Cell(4, 0), Cell(5, 0))
        self.assertEqual(ctl.desired(three_off).kind, CommandKind.CANCEL)
        ctl2 = ExecutionController()
        running(ctl2, long)
        self.assertEqual(ctl2.desired((Cell(3, 0), Cell(4, 0), Cell(4, 1))).kind, CommandKind.CANCEL)

    def test_replan_cancels_first_and_sends_only_after_termination(self):
        ctl = ExecutionController()
        running(ctl)
        self.assertEqual(ctl.desired(B).kind, CommandKind.CANCEL)
        self.assertEqual(ctl.state, ExecState.CANCELLING)
        self.assertEqual(ctl.sent_count, 1)  # nothing new sent while old goal may still run
        cmd = ctl.on_result(GoalOutcome.CANCELED)
        self.assertEqual((cmd.kind, cmd.cells), (CommandKind.SEND, B))
        self.assertEqual(ctl.sent_count, 2)

    def test_latest_pending_route_wins_during_cancel(self):
        ctl = ExecutionController()
        running(ctl)
        ctl.desired(B)
        third = (Cell(5, 5),)
        self.assertEqual(ctl.desired(third).kind, CommandKind.NONE)
        self.assertEqual(ctl.on_result(GoalOutcome.CANCELED).cells, third)

    def test_stop_during_cancel_drops_pending_replacement(self):
        ctl = ExecutionController()
        running(ctl)
        ctl.desired(B)
        self.assertEqual(ctl.desired(None).kind, CommandKind.NONE)  # already cancelling
        self.assertEqual(ctl.on_result(GoalOutcome.CANCELED).kind, CommandKind.NONE)
        self.assertEqual(ctl.state, ExecState.IDLE)
        self.assertEqual(ctl.sent_count, 1)

    def test_no_route_cancels_active_goal_and_sends_nothing(self):
        ctl = ExecutionController()
        running(ctl)
        self.assertEqual(ctl.desired(None).kind, CommandKind.CANCEL)
        self.assertEqual(ctl.on_result(GoalOutcome.CANCELED).kind, CommandKind.NONE)
        self.assertEqual(ctl.desired(None).kind, CommandKind.NONE)
        self.assertEqual(ctl.sent_count, 1)

    def test_stop_while_dispatching_cancels(self):
        ctl = ExecutionController()
        ctl.desired(A)
        self.assertEqual(ctl.desired(None).kind, CommandKind.CANCEL)

    def test_rejected_goal_returns_to_idle_and_can_retry(self):
        ctl = ExecutionController()
        ctl.desired(A)
        ctl.on_goal_response(False)
        self.assertEqual(ctl.state, ExecState.IDLE)
        self.assertEqual(ctl.desired(A).kind, CommandKind.SEND)

    def test_repeated_failures_latch_fault_until_reset(self):
        ctl = ExecutionController(max_consecutive_failures=2)
        for _ in range(2):
            ctl.desired(A)
            ctl.on_goal_response(False)
        self.assertEqual(ctl.state, ExecState.FAULT)
        cmd = ctl.desired(A)
        self.assertEqual((cmd.kind, cmd.reason_code), (CommandKind.NONE, "execution_fault_latched"))
        ctl.reset()
        self.assertEqual(ctl.desired(A).kind, CommandKind.SEND)

    def test_goal_accepted_after_cancel_request_is_cancelled_again(self):
        ctl = ExecutionController()
        ctl.desired(A)
        ctl.desired(B)  # replan while still dispatching -> cancel + pending
        self.assertEqual(ctl.state, ExecState.CANCELLING)
        self.assertEqual(ctl.on_goal_response(True).kind, CommandKind.CANCEL)
        self.assertEqual(ctl.state, ExecState.CANCELLING)
        self.assertEqual(ctl.on_result(GoalOutcome.CANCELED).cells, B)

    def test_goal_rejected_after_cancel_request_sends_pending(self):
        ctl = ExecutionController()
        ctl.desired(A)
        ctl.desired(B)
        self.assertEqual(ctl.on_goal_response(False).cells, B)

    def test_controller_failure_clears_pending(self):
        ctl = ExecutionController()
        running(ctl)
        ctl.desired(B)
        self.assertEqual(ctl.on_result(GoalOutcome.FAILED).kind, CommandKind.NONE)
        self.assertEqual(ctl.state, ExecState.IDLE)
        self.assertIsNone(ctl.pending_cells)

    def test_events_in_wrong_state_are_errors_not_silent(self):
        ctl = ExecutionController()
        with self.assertRaises(RuntimeError):
            ctl.on_goal_response(True)
        with self.assertRaises(RuntimeError):
            ctl.on_result(GoalOutcome.SUCCEEDED)

    def test_invalid_arguments(self):
        with self.assertRaises(ValueError):
            ExecutionController(max_consecutive_failures=0)
        with self.assertRaises(ValueError):
            ExecutionController().desired(())
        with self.assertRaises(ValueError):
            ExecutionController(rejoin_cells=-1)


class StaleInputTests(unittest.TestCase):
    def test_missing_future_and_old_stamps_are_stale(self):
        stamps = {"pose": 99.5, "map": None, "forecast": 90.0, "clock_skew": 101.0}
        self.assertEqual(stale_inputs(100.0, stamps, 2.0), ["clock_skew", "forecast", "map"])

    def test_fresh_inputs(self):
        self.assertEqual(stale_inputs(10.0, {"pose": 9.0}, 2.0), [])

    def test_invalid_age(self):
        with self.assertRaises(ValueError):
            stale_inputs(1.0, {}, 0.0)


if __name__ == "__main__":
    unittest.main()


class HysteresisTests(unittest.TestCase):
    def test_remaining_route(self):
        from aegisnav_core.execution import remaining_route
        self.assertEqual(remaining_route(A, Cell(1, 0)), A[1:])
        self.assertIsNone(remaining_route(A, Cell(9, 9)))
        # One cell off the route, beside cells 1 and 2: resume from the further one.
        self.assertEqual(remaining_route(A, Cell(1, 1)), A[2:])

    def test_keep_active_unless_materially_better(self):
        from aegisnav_core.execution import keep_active_route
        from aegisnav_core.models import RoutePlan
        active = RoutePlan(A, True, "ok", total_cost=10.0)
        slightly = RoutePlan(B[:1] + A[1:], True, "ok", total_cost=9.5)
        much = RoutePlan(B[:1] + A[1:], True, "ok", total_cost=7.0)
        other_goal = RoutePlan(B, True, "ok", total_cost=1.0)
        self.assertTrue(keep_active_route(active, slightly, min_relative_improvement=0.15))
        self.assertFalse(keep_active_route(active, much, min_relative_improvement=0.15))
        self.assertFalse(keep_active_route(active, other_goal, min_relative_improvement=0.15))
        self.assertFalse(keep_active_route(RoutePlan((), False, "route_crosses_hard_threshold"), slightly, min_relative_improvement=0.15))
        with self.assertRaises(ValueError):
            keep_active_route(active, slightly, min_relative_improvement=1.0)
