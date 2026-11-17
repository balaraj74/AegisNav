import unittest

from aegisnav_core.models import Cell, GridMap, HazardFrame, PlannerConfig
from aegisnav_core.planning import RiskAwareAStar


class RiskAwarePlanningTests(unittest.TestCase):
    def test_finds_path_in_open_grid(self):
        grid = GridMap(8, 6, resolution_m=0.5)
        zeros = (0.0,) * grid.size
        uncertain = (0.1,) * grid.size
        frame = HazardFrame(grid.width, grid.height, zeros, uncertain)
        plan = RiskAwareAStar().plan(grid, [frame], Cell(0, 0), Cell(7, 5))
        self.assertTrue(plan.success, plan.reason)
        self.assertEqual(plan.cells[0], Cell(0, 0))
        self.assertEqual(plan.cells[-1], Cell(7, 5))
        self.assertGreater(plan.distance_m, 0)

    def test_avoids_critical_cell(self):
        grid = GridMap(7, 5)
        risk = [0.0] * grid.size
        for x in range(1, 6):
            risk[grid.index(Cell(x, 2))] = 0.96
        frame = HazardFrame(grid.width, grid.height, tuple(risk), (0.0,) * grid.size)
        plan = RiskAwareAStar().plan(
            grid, [frame], Cell(0, 2), Cell(6, 2),
            PlannerConfig(hazard_weight=0.0, uncertainty_weight=0.0, hard_risk_threshold=0.9),
        )
        self.assertTrue(plan.success)
        self.assertNotIn(Cell(3, 2), plan.cells)

    def test_rejects_blocked_goal(self):
        grid = GridMap(4, 3, blocked=frozenset({11}))
        zero = (0.0,) * grid.size
        frame = HazardFrame(grid.width, grid.height, zero, zero)
        plan = RiskAwareAStar().plan(grid, [frame], Cell(0, 0), Cell(3, 2))
        self.assertFalse(plan.success)
        self.assertEqual(plan.reason, "start_or_goal_blocked")

    def test_diagonal_corner_cutting_is_forbidden(self):
        grid = GridMap(2, 2, blocked=frozenset({1, 2}))
        neighbors = list(grid.neighbors(Cell(0, 0), diagonal=True))
        self.assertEqual(neighbors, [])


if __name__ == "__main__":
    unittest.main()
