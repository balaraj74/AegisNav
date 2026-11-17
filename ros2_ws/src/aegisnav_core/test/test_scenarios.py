import unittest

from aegisnav_core.forecast import HazardForecaster
from aegisnav_core.mission import MissionPolicy, MissionSafetySupervisor
from aegisnav_core.models import Cell, HazardSource, MissionAction, PlannerConfig
from aegisnav_core.planning import RiskAwareAStar
from aegisnav_core.scenarios import SCENARIOS, scenario_by_name


def passage_widths_ok(scn):
    g, k = scn.grid, scn.min_passage_cells
    free = lambda x, y: g.contains(Cell(x, y)) and not g.is_blocked(Cell(x, y))
    bad = []
    for i in range(g.size):
        c = g.cell(i)
        if g.is_blocked(c):
            continue
        for dx, dy in ((1, 0), (0, 1)):
            n, x, y = 1, c.x + dx, c.y + dy
            while free(x, y):
                n, x, y = n + 1, x + dx, y + dy
            x, y = c.x - dx, c.y - dy
            while free(x, y):
                n, x, y = n + 1, x - dx, y - dy
            if n < k:
                bad.append((c, (dx, dy), n))
    return bad


class ScenarioGeometryTests(unittest.TestCase):
    def test_every_scenario_respects_its_min_passage(self):
        for name in SCENARIOS:
            scn = scenario_by_name(name)
            self.assertEqual(passage_widths_ok(scn), [], name)

    def test_mission_cells_free_victims_and_props_blocked(self):
        for name in SCENARIOS:
            scn = scenario_by_name(name)
            g = scn.grid
            for c in (scn.start, scn.base, *scn.refuges, *scn.mission_goals):
                self.assertFalse(g.is_blocked(c), (name, c))
            for c in (*scn.victims, *(c for c, _ in scn.props)):
                self.assertTrue(g.is_blocked(c), (name, c))
            for src in scn.sources:
                self.assertFalse(g.is_blocked(src.cell), (name, src))

    def test_unknown_scenario(self):
        with self.assertRaises(ValueError):
            scenario_by_name("nope")


class RescueBuildingMissionTests(unittest.TestCase):
    def setUp(self):
        self.scn = scenario_by_name("rescue_building")
        self.frames = HazardForecaster().forecast(self.scn.grid, self.scn.initial_risk, self.scn.initial_uncertainty,
                                                  self.scn.sources, self.scn.forecast_config).frames
        self.cfg = PlannerConfig(nominal_speed_mps=0.7)

    def test_every_leg_has_a_route(self):
        stops = (self.scn.base, *self.scn.mission_goals, self.scn.base)
        for a, b in zip(stops, stops[1:]):
            plan = RiskAwareAStar().plan(self.scn.grid, self.frames, a, b, self.cfg, time_step_s=2.0)
            self.assertTrue(plan.success, (a, b, plan.reason))

    def test_first_leg_is_feasible_mission(self):
        sup = MissionSafetySupervisor(policy=MissionPolicy(planner=self.cfg))
        r = sup.assess(self.scn.grid, self.frames, self.scn.base, self.scn.mission_goals[0], self.scn.base,
                       self.scn.refuges, 0.95, time_step_s=2.0)
        self.assertEqual(r.action, MissionAction.CONTINUE_TO_TARGET, (r.reason_code, r.explanation))

    def test_scripted_fire_blocks_first_leg_and_a_detour_exists(self):
        # The demo is only meaningful if the scripted ignition lands on the route the robot is
        # following and a safe detour remains. Planner settings: the rescue_building preset in
        # aegisnav_bringup/config/worlds.yaml.
        cfg = PlannerConfig(nominal_speed_mps=0.6, wall_clearance_cells=2, wall_clearance_penalty_m=1.0,
                            straight_near_obstacles=True, inflate_obstacles_cells=1)
        goal = self.scn.mission_goals[0]
        before = RiskAwareAStar().plan(self.scn.grid, self.frames, self.scn.base, goal, cfg, time_step_s=2.0)
        ignitions = self.scn.scripted_ignitions
        t_ign = max(t for t, _, _ in ignitions)
        fires = [c for _, c, _ in ignitions]

        def gap(route):
            return min(max(abs(c.x - f.x), abs(c.y - f.y)) for c in route for f in fires)

        self.assertTrue(before.success)
        self.assertLessEqual(gap(before.cells), 2)
        # Robot position at ignition if it had started moving at t = 0 (live start-up is ~3 s later).
        robot = before.cells[min(len(before.cells) - 1, int(t_ign / before.estimated_time_s * len(before.cells)))]
        frames = HazardForecaster().forecast(
            self.scn.grid, self.scn.initial_risk, self.scn.initial_uncertainty,
            (*self.scn.sources, *(HazardSource(c, i, "scripted") for _, c, i in ignitions)),
            self.scn.forecast_config).frames
        after = RiskAwareAStar().plan(self.scn.grid, frames, robot, goal, cfg, time_step_s=2.0)
        self.assertTrue(after.success, after.reason)
        self.assertGreaterEqual(gap(after.cells), 5)

    def test_confidence_drop_comes_after_both_victims(self):
        # Observed headless: victim 2 reached ~86 s after the first pose; the drop must not cut the
        # outbound mission short, and must come before the robot is home (~125 s).
        (t_drop, value), = self.scn.scripted_confidence
        self.assertTrue(90.0 <= t_drop <= 120.0)
        self.assertLess(value, 0.55)


class RescueCompoundMissionTests(unittest.TestCase):
    def test_every_inspection_leg_is_a_feasible_mission(self):
        # Planner settings: the rescue_compound preset in aegisnav_bringup/config/worlds.yaml.
        scn = scenario_by_name("rescue_compound")
        cfg = PlannerConfig(nominal_speed_mps=0.6, wall_clearance_cells=2, wall_clearance_penalty_m=1.0,
                            straight_near_obstacles=True, inflate_obstacles_cells=1)
        frames = HazardForecaster().forecast(scn.grid, scn.initial_risk, scn.initial_uncertainty, scn.sources,
                                             scn.forecast_config).frames
        sup = MissionSafetySupervisor(policy=MissionPolicy(planner=cfg))
        stops = (scn.base, *scn.mission_goals, scn.base)
        self.assertEqual(len(scn.mission_goals), 5)
        for a, b in zip(stops, stops[1:]):
            r = sup.assess(scn.grid, frames, a, b, scn.base, scn.refuges, 0.95, time_step_s=2.0)
            self.assertEqual(r.action, MissionAction.CONTINUE_TO_TARGET, (a, b, r.reason_code))


if __name__ == "__main__":
    unittest.main()
