import unittest

from aegisnav_core.mission import MissionPolicy, MissionSafetySupervisor
from aegisnav_core.models import Cell, ForecastConfig, GridMap, HazardFrame, MissionAction, PlannerConfig


class MissionSupervisorTests(unittest.TestCase):
    def setUp(self):
        self.grid = GridMap(7, 5, resolution_m=0.5)
        self.zero = (0.0,) * self.grid.size
        self.frames = [HazardFrame(self.grid.width, self.grid.height, self.zero, (0.05,) * self.grid.size, i, float(i)) for i in range(101)]
        self.start, self.target, self.base = Cell(0, 2), Cell(6, 2), Cell(0, 2)
        self.supervisor = MissionSafetySupervisor(policy=MissionPolicy(
            planner=PlannerConfig(hazard_weight=1.0, uncertainty_weight=0.1, hard_risk_threshold=0.95),
            max_outbound_exposure=100.0, max_return_exposure=100.0,
        ))

    def test_feasible_mission_continues(self):
        result = self.supervisor.assess(self.grid, self.frames, self.start, self.target, self.base, [Cell(1, 1)], 0.95)
        self.assertEqual(result.action, MissionAction.CONTINUE_TO_TARGET)
        self.assertTrue(result.safe_return_feasible)

    def test_degraded_perception_changes_mode(self):
        result = self.supervisor.assess(self.grid, self.frames, self.start, self.target, self.base, [Cell(1, 1)], 0.45)
        self.assertEqual(result.action, MissionAction.SLOW_AND_REASSESS)
        self.assertIsNotNone(result.refuge_route)

    def test_critical_confidence_prefers_refuge(self):
        result = self.supervisor.assess(self.grid, self.frames, self.start, self.target, self.base, [Cell(1, 1)], 0.1)
        self.assertEqual(result.action, MissionAction.SEEK_REFUGE)

    def test_empty_forecast_fails_closed(self):
        result = self.supervisor.assess(self.grid, [], self.start, self.target, self.base, [], 1.0)
        self.assertEqual(result.action, MissionAction.ABORT)


    def test_critical_current_cell_never_continues_to_target(self):
        risk = list(self.zero)
        risk[self.grid.index(self.start)] = 0.99
        frames = [
            HazardFrame(self.grid.width, self.grid.height, tuple(risk), (0.05,) * self.grid.size, i, float(i))
            for i in range(101)
        ]
        result = self.supervisor.assess(self.grid, frames, self.start, self.target, self.base, [Cell(1, 1)], 0.95)
        self.assertNotEqual(result.action, MissionAction.CONTINUE_TO_TARGET)
        self.assertIn(result.reason_code, {"current_cell_critical", "current_cell_critical_no_exit"})

    def test_target_mission_is_rejected_when_forecast_horizon_is_too_short(self):
        result = self.supervisor.assess(self.grid, self.frames[:4], self.start, self.target, self.base, [], 0.95)
        self.assertNotEqual(result.action, MissionAction.CONTINUE_TO_TARGET)
        self.assertFalse(result.forecast_horizon_sufficient)


if __name__ == "__main__":
    unittest.main()
