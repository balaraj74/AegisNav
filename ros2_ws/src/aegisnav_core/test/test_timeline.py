import unittest

from aegisnav_core.simulation import seeded_dynamic_scenario
from aegisnav_core.timeline import ScenarioTimeline


class TimelineTests(unittest.TestCase):
    def test_events_switch_at_configured_times(self):
        ds = seeded_dynamic_scenario(7)
        tl = ScenarioTimeline(ds)
        before = tl.snapshot(ds.ignition_time_s - 0.01)
        after = tl.snapshot(ds.ignition_time_s)
        self.assertEqual(before.ignited_cells, ())
        self.assertEqual(after.ignited_cells[0][0], ds.ignition_sources[0].cell)
        self.assertEqual(tl.snapshot(ds.confidence_drop_time_s - 0.01).confidence, ds.confidence_before)
        self.assertEqual(tl.snapshot(ds.confidence_drop_time_s).confidence, ds.confidence_after)

    def test_negative_time_rejected(self):
        with self.assertRaises(ValueError):
            ScenarioTimeline(seeded_dynamic_scenario(0)).snapshot(-1.0)

    def test_same_seed_same_timeline(self):
        a, b = ScenarioTimeline(seeded_dynamic_scenario(5)), ScenarioTimeline(seeded_dynamic_scenario(5))
        self.assertEqual(a.snapshot(100.0), b.snapshot(100.0))


class NextGoalTests(unittest.TestCase):
    def test_switches_to_base_only_when_holding_at_target_under_continue(self):
        from aegisnav_core.models import Cell
        from aegisnav_core.timeline import next_goal
        t, b = Cell(9, 9), Cell(1, 1)
        self.assertEqual(next_goal(t, (t,), b, at_destination=True, action="continue_to_target"), b)
        self.assertEqual(next_goal(t, (t,), b, at_destination=False, action="continue_to_target"), t)
        self.assertEqual(next_goal(t, (t,), b, at_destination=True, action="seek_refuge"), t)
        self.assertEqual(next_goal(b, (t,), b, at_destination=True, action="continue_to_target"), b)

    def test_sequence_of_victims_then_base(self):
        from aegisnav_core.models import Cell
        from aegisnav_core.timeline import next_goal
        v1, v2, b = Cell(5, 5), Cell(9, 1), Cell(0, 0)
        self.assertEqual(next_goal(v1, (v1, v2), b, at_destination=True, action="continue_to_target"), v2)
        self.assertEqual(next_goal(v2, (v1, v2), b, at_destination=True, action="continue_to_target"), b)


class ScriptTimelineTests(unittest.TestCase):
    def test_schedule(self):
        from aegisnav_core.models import Cell
        from aegisnav_core.timeline import ScriptTimeline
        tl = ScriptTimeline(((10.0, Cell(1, 1), 0.9),), ((20.0, 0.4),), default_confidence=0.95)
        self.assertEqual(tl.snapshot(9.9).ignited_cells, ())
        self.assertEqual(tl.snapshot(10.0).ignited_cells, ((Cell(1, 1), 0.9),))
        self.assertEqual((tl.snapshot(19.0).confidence, tl.snapshot(20.0).confidence), (0.95, 0.4))

    def test_invalid(self):
        from aegisnav_core.models import Cell
        from aegisnav_core.timeline import ScriptTimeline
        with self.assertRaises(ValueError):
            ScriptTimeline(((1.0, Cell(0, 0), 1.5),), ())
        with self.assertRaises(ValueError):
            ScriptTimeline((), ((1.0, -0.1),))


class TriageTests(unittest.TestCase):
    def setUp(self):
        from aegisnav_core.models import Cell, GridMap, HazardFrame
        self.Cell = Cell
        self.grid = GridMap(9, 3)
        risk = [0.0] * self.grid.size
        risk[self.grid.index(Cell(6, 1))] = 0.5  # hazardous (but passable) cell beside goal C
        self.frames = [HazardFrame(9, 3, tuple(risk), (0.1,) * self.grid.size, k, float(k)) for k in range(30)]

    def triage(self, remaining, frames=None):
        from aegisnav_core.models import PlannerConfig
        from aegisnav_core.timeline import triage_next_goal
        return triage_next_goal(self.Cell(4, 1), remaining, self.Cell(0, 0), self.grid, frames or self.frames,
                                PlannerConfig(), time_step_s=1.0)

    def test_lowest_exposure_reachable_goal_first(self):
        C = self.Cell
        self.assertEqual(self.triage((C(7, 1), C(1, 1))).goal, C(1, 1))  # C(7,1) route crosses the hazard

    def test_unreachable_goals_are_reported_and_base_is_the_fallback(self):
        from aegisnav_core.models import HazardFrame
        C, g = self.Cell, self.grid
        risk = [1.0 if g.cell(i).x in (3, 5) else 0.0 for i in range(g.size)]  # walls of fire both sides
        frames = [HazardFrame(9, 3, tuple(risk), (0.1,) * g.size, k, float(k)) for k in range(30)]
        result = self.triage((C(7, 1), C(1, 1)), frames)
        self.assertEqual(result.goal, C(0, 0))
        self.assertEqual(result.unreachable, (C(7, 1), C(1, 1)))


class ConfidenceHysteresisTests(unittest.TestCase):
    def test_latches_until_resume_threshold(self):
        from aegisnav_core.mission import ConfidenceHysteresis
        h = ConfidenceHysteresis(0.55, 0.65)
        self.assertEqual(h.update(0.9), 0.9)
        self.assertLess(h.update(0.5), 0.55)
        self.assertLess(h.update(0.6), 0.55)  # recovered above continue but below resume: still held
        self.assertTrue(h.latched)
        self.assertEqual(h.update(0.7), 0.7)
        self.assertFalse(h.latched)
        with self.assertRaises(ValueError):
            ConfidenceHysteresis(0.6, 0.5)
        with self.assertRaises(ValueError):
            h.update(1.5)


if __name__ == "__main__":
    unittest.main()
