import unittest

from aegisnav_core.forecast import HazardForecaster
from aegisnav_core.mission import MissionPolicy, MissionSafetySupervisor
from aegisnav_core.models import (
    Cell, ForecastConfig, GridMap, HazardFrame, HazardSource, MissionAction, MissionAssessment,
    PlannerConfig, RoutePlan,
)
from aegisnav_core.planning import RiskAwareAStar, evaluate_route


def frames(grid, risk=None, n=60):
    risk = risk or (0.0,) * grid.size
    return [HazardFrame(grid.width, grid.height, tuple(risk), (0.0,) * grid.size, i, float(i)) for i in range(n)]


def wall_grid():
    # 5x3 grid split by a full-height wall at x=2.
    return GridMap(5, 3, 0.5, frozenset(y * 5 + 2 for y in range(3)))


class PlannerRegressionTests(unittest.TestCase):
    def test_blocked_start(self):
        g = wall_grid()
        r = RiskAwareAStar().plan(g, frames(g), Cell(2, 1), Cell(4, 1))
        self.assertEqual((r.success, r.reason), (False, "start_or_goal_blocked"))

    def test_goal_outside_map(self):
        g = wall_grid()
        self.assertEqual(RiskAwareAStar().plan(g, frames(g), Cell(0, 0), Cell(9, 9)).reason, "start_or_goal_outside_map")

    def test_no_path_behind_wall(self):
        g = wall_grid()
        r = RiskAwareAStar().plan(g, frames(g), Cell(0, 1), Cell(4, 1))
        self.assertEqual((r.success, r.reason), (False, "no_safe_path"))

    def test_forecast_dimension_mismatch_raises(self):
        g = GridMap(4, 4)
        other = GridMap(3, 3)
        with self.assertRaises(ValueError):
            RiskAwareAStar().plan(g, frames(other), Cell(0, 0), Cell(3, 3))

    def test_missing_forecast_fails_closed(self):
        g = GridMap(3, 3)
        self.assertEqual(RiskAwareAStar().plan(g, [], Cell(0, 0), Cell(2, 2)).reason, "missing_forecast")

    def test_high_risk_corridor_is_rejected_not_traversed(self):
        g = GridMap(5, 1)
        risk = [0.0, 0.0, 0.95, 0.0, 0.0]
        r = RiskAwareAStar().plan(g, frames(g, risk), Cell(0, 0), Cell(4, 0), PlannerConfig(hard_risk_threshold=0.9))
        self.assertFalse(r.success)

    def test_planning_is_deterministic(self):
        g = GridMap(8, 8)
        risk = [0.0] * g.size
        risk[g.index(Cell(4, 4))] = 0.5
        a = RiskAwareAStar().plan(g, frames(g, risk), Cell(0, 0), Cell(7, 7))
        b = RiskAwareAStar().plan(g, frames(g, risk), Cell(0, 0), Cell(7, 7))
        self.assertEqual(a, b)

    def test_invalid_planner_config(self):
        for kwargs in ({"hazard_weight": -1}, {"hard_risk_threshold": 0.0}, {"nominal_speed_mps": 0}, {"max_expansions": 0}):
            with self.assertRaises(ValueError):
                PlannerConfig(**kwargs)

    def test_clearance_penalty_prefers_lane_away_from_wall(self):
        # 6 wide x 4 tall; wall along y=0 row except it is the map edge row -> blocked row y=0.
        g0 = GridMap(6, 4)
        g = GridMap(6, 4, 1.0, frozenset(g0.index(Cell(x, 0)) for x in range(6)))
        hug = RiskAwareAStar().plan(g, frames(g), Cell(0, 1), Cell(5, 1), PlannerConfig(wall_clearance_penalty_m=0.0))
        clear = RiskAwareAStar().plan(g, frames(g), Cell(0, 1), Cell(5, 1), PlannerConfig(wall_clearance_penalty_m=2.0))
        self.assertTrue(all(c.y == 1 for c in hug.cells))
        self.assertTrue(any(c.y == 2 for c in clear.cells))

    def test_invalid_clearance_penalty(self):
        with self.assertRaises(ValueError):
            PlannerConfig(wall_clearance_penalty_m=-0.1)
        with self.assertRaises(ValueError):
            PlannerConfig(wall_clearance_cells=0)

    def test_straight_near_obstacles_forbids_diagonals_by_walls(self):
        # 2-cell-wide doorway in a wall at y=2; straight rule => no diagonal step touching cells near walls.
        g0 = GridMap(7, 5)
        g = GridMap(7, 5, 1.0, frozenset(g0.index(Cell(x, 2)) for x in (0, 1, 4, 5, 6)))
        cfg = PlannerConfig(straight_near_obstacles=True, wall_clearance_penalty_m=0.0, wall_clearance_cells=1)
        plan = RiskAwareAStar().plan(g, frames(g), Cell(0, 0), Cell(6, 4), cfg)
        self.assertTrue(plan.success)
        from aegisnav_core.planning import clearance_weights
        near = clearance_weights(g, 1)
        for a, b in zip(plan.cells, plan.cells[1:]):
            if a.x != b.x and a.y != b.y:
                self.assertNotIn(g.index(a), near)
                self.assertNotIn(g.index(b), near)
        self.assertEqual(evaluate_route(g, frames(g), (Cell(1, 1), Cell(2, 2)), cfg).reason, "route_step_illegal")

    def test_inflation_keeps_robot_centre_off_wall_cells(self):
        # Corridor 5 cells tall between walls at y=0 and y=6: with inflation 1, only y=2..4 are usable.
        g0 = GridMap(8, 7)
        walls = {g0.index(Cell(x, y)) for x in range(8) for y in (0, 6)}
        g = GridMap(8, 7, 1.0, frozenset(walls))
        cfg = PlannerConfig(inflate_obstacles_cells=1, wall_clearance_penalty_m=0.0)
        plan = RiskAwareAStar().plan(g, frames(g), Cell(0, 1), Cell(7, 1), cfg)  # start/goal may touch walls
        self.assertTrue(plan.success)
        self.assertTrue(all(2 <= c.y <= 4 for c in plan.cells[1:-1]))
        self.assertEqual(evaluate_route(g, frames(g), tuple(Cell(x, 1) for x in range(8)), cfg).reason, "route_step_illegal")
        with self.assertRaises(ValueError):
            PlannerConfig(inflate_obstacles_cells=4)

    def test_clearance_weights_radius(self):
        from aegisnav_core.planning import clearance_weights
        g0 = GridMap(5, 5)
        g = GridMap(5, 5, 1.0, frozenset({g0.index(Cell(0, 2))}))
        w1, w2 = clearance_weights(g, 1), clearance_weights(g, 2)
        self.assertEqual((w1.get(g.index(Cell(1, 2))), w1.get(g.index(Cell(2, 2)))), (1, None))
        self.assertEqual((w2.get(g.index(Cell(1, 2))), w2.get(g.index(Cell(2, 2)))), (2, 1))

    def test_cached_tables_match_grid_definitions(self):
        # The planner's per-grid tables must stay equivalent to GridMap.neighbors / clearance_weights,
        # including neighbour order (it decides ties between equal-cost routes).
        from aegisnav_core.planning import _adjacency, _clearance, clearance_weights
        from aegisnav_core.scenarios import rescue_building_scenario
        g = rescue_building_scenario().grid
        for diagonal in (True, False):
            table = _adjacency(g, diagonal)
            for idx in range(g.size):
                expected = () if idx in g.blocked else tuple(
                    (g.index(n), d) for n, d in g.neighbors(g.cell(idx), diagonal=diagonal))
                self.assertEqual(table[idx], expected)
        for radius in (1, 2, 3):
            self.assertEqual(_clearance(g, radius), clearance_weights(g, radius))

    def test_heuristic_is_admissible_and_consistent(self):
        # The cost-to-go heuristic must never exceed the real cost from a cell (admissible) and must
        # satisfy h(u) <= cost(u -> v) + h(v) for every move (consistent), or A* loses optimality.
        import math
        from aegisnav_core.planning import _adjacency, _clearance, _cost_to_go, _edge_cost, _min_rates
        from aegisnav_core.scenarios import rescue_building_scenario
        scn = rescue_building_scenario()
        g = scn.grid
        cfg = PlannerConfig(nominal_speed_mps=0.6, wall_clearance_cells=2, wall_clearance_penalty_m=1.0)
        fr = HazardForecaster().forecast(g, scn.initial_risk, scn.initial_uncertainty, scn.sources,
                                         scn.forecast_config).frames
        goal = scn.mission_goals[0]
        near = _clearance(g, cfg.wall_clearance_cells)
        h = _cost_to_go(g, _adjacency(g, True), _min_rates(fr, cfg.hazard_weight, cfg.uncertainty_weight),
                        g.index(goal), cfg, near)
        risk, unc = [f.risk for f in fr], [f.uncertainty for f in fr]
        for u in range(0, g.size, 5):
            for v, length in _adjacency(g, True)[u]:
                for t in (0.0, 13.0, 90.0):
                    step = _edge_cost(risk, unc, t, v, length * g.resolution_m, cfg, 2.0, clearance_weight=near.get(v, 0))
                    if step is not None:
                        self.assertLessEqual(h[u], step[2] + h[v] + 1e-9)
        for start in (scn.base, scn.refuges[0], Cell(36, 3)):
            plan = RiskAwareAStar().plan(g, fr, start, goal, cfg, time_step_s=2.0)
            self.assertTrue(plan.success)
            self.assertLessEqual(h[g.index(start)], plan.total_cost + 1e-9)
            self.assertLess(h[g.index(start)], math.inf)

    def test_hazard_standoff_keeps_the_route_away_and_allows_escape(self):
        # 13 x 11 open room, a fire (risk 0.95) in the middle of the direct line from (0,5) to (12,5).
        g = GridMap(13, 11)
        risk = [0.0] * g.size
        risk[g.index(Cell(6, 5))] = 0.95
        fr = [HazardFrame(13, 11, tuple(risk), (0.1,) * g.size, k, float(k)) for k in range(120)]
        start, goal = Cell(0, 5), Cell(12, 5)

        def nearest(plan):
            return min(max(abs(c.x - 6), abs(c.y - 5)) for c in plan.cells)

        base = PlannerConfig(hazard_weight=0.0, uncertainty_weight=0.0)  # distance only: would hug the fire
        passing = RiskAwareAStar().plan(g, fr, start, goal, base, time_step_s=1.0)
        self.assertTrue(passing.success)
        self.assertLessEqual(nearest(passing), 1)
        for radius in (1, 2, 3):
            cfg = PlannerConfig(hazard_weight=0.0, uncertainty_weight=0.0, hazard_standoff_cells=radius)
            plan = RiskAwareAStar().plan(g, fr, start, goal, cfg, time_step_s=1.0)
            self.assertTrue(plan.success, radius)
            self.assertGreater(nearest(plan), radius, radius)  # never enters the zone
            self.assertTrue(evaluate_route(g, fr, plan.cells, cfg, time_step_s=1.0).success)
            self.assertFalse(evaluate_route(g, fr, passing.cells, cfg, time_step_s=1.0).success)  # the old route is rejected
        # Starting inside the zone: the robot may move within it and out of it.
        cfg = PlannerConfig(hazard_weight=0.0, uncertainty_weight=0.0, hazard_standoff_cells=2)
        self.assertTrue(RiskAwareAStar().plan(g, fr, Cell(5, 5), Cell(0, 5), cfg, time_step_s=1.0).success)
        # A goal inside the zone cannot be entered from outside: no route (the mission policy then handles it).
        self.assertFalse(RiskAwareAStar().plan(g, fr, Cell(0, 5), Cell(5, 5), cfg, time_step_s=1.0).success)
        # A room the zone closes completely has no route either.
        g2 = GridMap(11, 5)
        risk2 = [0.0] * g2.size
        risk2[g2.index(Cell(5, 2))] = 0.95
        fr2 = [HazardFrame(11, 5, tuple(risk2), (0.1,) * g2.size, k, float(k)) for k in range(60)]
        self.assertFalse(RiskAwareAStar().plan(g2, fr2, Cell(0, 2), Cell(10, 2), cfg, time_step_s=1.0).success)

    def test_hazard_standoff_does_not_spread_through_walls(self):
        # A wall at x=5 with a fire right behind it (x=6): the corridor cells at x=4 are within 2 cells by
        # distance but not through free space, so the standoff must not close them.
        g0 = GridMap(11, 3)
        g = GridMap(11, 3, 1.0, frozenset({g0.index(Cell(5, 0)), g0.index(Cell(5, 1)), g0.index(Cell(5, 2))}))
        risk = [0.0] * g.size
        risk[g.index(Cell(6, 1))] = 0.95
        fr = [HazardFrame(11, 3, tuple(risk), (0.1,) * g.size, k, float(k)) for k in range(30)]
        from aegisnav_core.planning import _standoff_zones
        zone = _standoff_zones(g, fr, 2, 0.92)[0]
        self.assertIn(g.index(Cell(7, 1)), zone)
        self.assertNotIn(g.index(Cell(4, 1)), zone)
        self.assertNotIn(g.index(Cell(5, 1)), zone)

    def test_hazard_standoff_validation(self):
        for bad in (-1, 5):
            with self.assertRaises(ValueError):
                PlannerConfig(hazard_standoff_cells=bad)
        self.assertEqual(PlannerConfig(hazard_standoff_cells=4).hazard_standoff_cells, 4)

    def test_invalid_time_step(self):
        g = GridMap(3, 3)
        with self.assertRaises(ValueError):
            RiskAwareAStar().plan(g, frames(g), Cell(0, 0), Cell(2, 2), time_step_s=0)


class EvaluateRouteTests(unittest.TestCase):
    def test_evaluation_reproduces_planner_cost(self):
        g = GridMap(8, 8)
        risk = [0.0] * g.size
        risk[g.index(Cell(3, 3))] = 0.6
        fr = frames(g, risk)
        plan = RiskAwareAStar().plan(g, fr, Cell(0, 0), Cell(7, 7))
        again = evaluate_route(g, fr, plan.cells)
        self.assertTrue(again.success)
        self.assertAlmostEqual(again.total_cost, plan.total_cost)
        self.assertAlmostEqual(again.distance_m, plan.distance_m)

    def test_evaluation_reports_exposure_through_a_risky_cell(self):
        g = GridMap(8, 8)
        risk = [0.0] * g.size
        risk[g.index(Cell(3, 3))] = 0.6
        diagonal = tuple(Cell(i, i) for i in range(8))  # passes through (3, 3)
        r = evaluate_route(g, frames(g, risk), diagonal)
        self.assertGreater(r.hazard_exposure, 0.0)
        self.assertAlmostEqual(r.max_risk_encountered, 0.6)

    def test_route_through_new_hazard_is_rejected(self):
        g = GridMap(5, 1)
        cells = tuple(Cell(x, 0) for x in range(5))
        hot = [0.0, 0.0, 0.95, 0.0, 0.0]
        self.assertEqual(evaluate_route(g, frames(g, hot), cells, PlannerConfig(hard_risk_threshold=0.9)).reason,
                         "route_crosses_hard_threshold")

    def test_corner_cut_and_blocked_steps_are_illegal(self):
        g = GridMap(3, 3, blocked=frozenset({GridMap(3, 3).index(Cell(1, 0))}))
        self.assertEqual(evaluate_route(g, frames(g), (Cell(0, 0), Cell(1, 1))).reason, "route_step_illegal")
        self.assertEqual(evaluate_route(g, frames(g), (Cell(1, 0),)).reason, "route_start_blocked")
        self.assertEqual(evaluate_route(g, frames(g), ()).reason, "empty_route")
        self.assertEqual(evaluate_route(g, [], (Cell(0, 0),)).reason, "missing_forecast")


class ForecastRegressionTests(unittest.TestCase):
    def test_source_in_blocked_cell_rejected(self):
        g = wall_grid()
        with self.assertRaises(ValueError):
            HazardForecaster().forecast(g, [0.0] * g.size, None, [HazardSource(Cell(2, 1), 1.0)], ForecastConfig(horizon_steps=2))

    def test_source_outside_grid_rejected(self):
        g = GridMap(3, 3)
        with self.assertRaises(ValueError):
            HazardForecaster().forecast(g, [0.0] * g.size, None, [HazardSource(Cell(5, 5), 1.0)], ForecastConfig(horizon_steps=2))

    def test_invalid_horizon_and_step(self):
        for kwargs in ({"horizon_steps": 0}, {"time_step_s": 0.0}, {"spread_rate": 1.5}, {"wind_direction": (1, 1)}):
            with self.assertRaises(ValueError):
                ForecastConfig(**kwargs)

    def test_forecast_is_deterministic(self):
        g = GridMap(6, 6)
        args = (g, [0.0] * g.size, None, [HazardSource(Cell(2, 2), 0.9)], ForecastConfig(horizon_steps=10))
        self.assertEqual(HazardForecaster().forecast(*args), HazardForecaster().forecast(*args))


class MissionRegressionTests(unittest.TestCase):
    def setUp(self):
        self.policy = MissionPolicy(planner=PlannerConfig(hazard_weight=1.0, hard_risk_threshold=0.9),
                                    max_outbound_exposure=100.0, max_return_exposure=100.0)
        self.sup = MissionSafetySupervisor(policy=self.policy)

    def test_target_unreachable_and_no_refuge_but_base_reachable_returns(self):
        g = wall_grid()
        r = self.sup.assess(g, frames(g), Cell(0, 1), Cell(4, 1), Cell(0, 0), [], 0.95)
        self.assertEqual((r.action, r.reason_code), (MissionAction.RETURN_TO_BASE, "target_route_unavailable"))
        self.assertIsNotNone(r.execution_route)

    def test_no_base_no_refuge_aborts_with_no_execution_route(self):
        g = wall_grid()
        r = self.sup.assess(g, frames(g), Cell(0, 1), Cell(4, 1), Cell(4, 0), [Cell(3, 0)], 0.95)
        self.assertEqual(r.action, MissionAction.ABORT)
        self.assertIsNone(r.execution_route)

    def test_unreachable_refuge_is_skipped_for_reachable_one(self):
        g = wall_grid()
        r = self.sup.assess(g, frames(g), Cell(0, 1), Cell(4, 1), Cell(0, 0), [Cell(4, 2), Cell(1, 2)], 0.1)
        self.assertEqual(r.action, MissionAction.SEEK_REFUGE)
        self.assertEqual(r.refuge_route.cells[-1], Cell(1, 2))

    def test_return_infeasible_after_target_selects_refuge(self):
        # Base cell turns critical after 8 s, before the robot could get back from the target.
        g = GridMap(7, 1)
        late = [HazardFrame(7, 1, (0.0,) * 7, (0.0,) * 7, i, float(i)) for i in range(8)]
        hot = (0.95, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        late += [HazardFrame(7, 1, hot, (0.0,) * 7, i, float(i)) for i in range(8, 100)]
        r = self.sup.assess(g, late, Cell(3, 0), Cell(6, 0), Cell(0, 0), [Cell(4, 0)], 0.95)
        self.assertEqual((r.action, r.reason_code), (MissionAction.SEEK_REFUGE, "return_route_unavailable"))
        self.assertEqual(r.execution_route.cells[-1], Cell(4, 0))

    def test_confidence_outside_range_rejected(self):
        g = GridMap(3, 3)
        with self.assertRaises(ValueError):
            self.sup.assess(g, frames(g), Cell(0, 0), Cell(2, 2), Cell(0, 0), [], 1.2)

    def test_policy_threshold_validation(self):
        with self.assertRaises(ValueError):
            MissionPolicy(minimum_continue_confidence=0.2, critical_confidence=0.4)
        with self.assertRaises(ValueError):
            MissionPolicy(max_outbound_exposure=-1.0)

    def test_every_decision_has_reason_and_explanation(self):
        g = wall_grid()
        for conf in (0.95, 0.45, 0.1):
            r = self.sup.assess(g, frames(g), Cell(0, 1), Cell(1, 1), Cell(0, 0), [Cell(1, 2)], conf)
            self.assertTrue(r.reason_code and r.explanation)


class ScenarioGeometryTests(unittest.TestCase):
    def test_no_one_cell_passages(self):
        # 0.4 m passages are not traversable by the 0.23 m robot under Nav2 (ADR-0005);
        # a point-robot planner must not be offered them.
        from aegisnav_core.scenarios import corridor_scenario
        g = corridor_scenario().grid
        blocked = lambda x, y: not g.contains(Cell(x, y)) or g.is_blocked(Cell(x, y))
        for i in range(g.size):
            c = g.cell(i)
            if g.is_blocked(c):
                continue
            self.assertFalse(blocked(c.x - 1, c.y) and blocked(c.x + 1, c.y), f"1-cell passage at {c}")
            self.assertFalse(blocked(c.x, c.y - 1) and blocked(c.x, c.y + 1), f"1-cell passage at {c}")


class ExecutionRouteTests(unittest.TestCase):
    ok = RoutePlan((Cell(0, 0), Cell(1, 0)), True, "ok")
    bad = RoutePlan((), False, "no_safe_path")

    def test_mapping(self):
        cases = [
            (MissionAssessment(MissionAction.CONTINUE_TO_TARGET, "r", "e", outbound=self.ok), self.ok),
            (MissionAssessment(MissionAction.SEEK_REFUGE, "r", "e", refuge_route=self.ok), self.ok),
            (MissionAssessment(MissionAction.SLOW_AND_REASSESS, "r", "e", refuge_route=self.ok), self.ok),
            (MissionAssessment(MissionAction.RETURN_TO_BASE, "r", "e", return_route=self.ok), self.ok),
            (MissionAssessment(MissionAction.ABORT, "r", "e", outbound=self.ok, return_route=self.ok), None),
            (MissionAssessment(MissionAction.CONTINUE_TO_TARGET, "r", "e", outbound=self.bad), None),
            (MissionAssessment(MissionAction.RETURN_TO_BASE, "r", "e"), None),
        ]
        for assessment, expected in cases:
            self.assertEqual(assessment.execution_route, expected)


if __name__ == "__main__":
    unittest.main()
