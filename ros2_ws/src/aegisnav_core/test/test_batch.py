import unittest

from aegisnav_core.batch import (
    Outcome, RunFacts, classify_outcome, mean_sd, paired_bootstrap_ci, randomized_ignitions, wilson_interval,
)
from aegisnav_core.scenarios import scenario_by_name


class RandomizedIgnitionTests(unittest.TestCase):
    def setUp(self):
        self.scn = scenario_by_name("rescue_compound")

    def test_deterministic_per_seed_and_different_across_seeds(self):
        self.assertEqual(randomized_ignitions(self.scn, 5), randomized_ignitions(self.scn, 5))
        self.assertNotEqual(randomized_ignitions(self.scn, 5), randomized_ignitions(self.scn, 6))

    def test_cells_are_free_away_from_start_goals_and_sources_and_times_sorted(self):
        for seed in range(25):
            ign = randomized_ignitions(self.scn, seed, count=3)
            self.assertEqual([t for t, _, _ in ign], sorted(t for t, _, _ in ign))
            for t, cell, intensity in ign:
                self.assertFalse(self.scn.grid.is_blocked(cell))
                self.assertGreater(max(abs(cell.x - self.scn.start.x), abs(cell.y - self.scn.start.y)), 8)
                for goal in self.scn.mission_goals:
                    self.assertGreater(max(abs(cell.x - goal.x), abs(cell.y - goal.y)), 3)
                self.assertNotIn(cell, {s.cell for s in self.scn.sources})
                self.assertTrue(20.0 <= t <= 240.0 and intensity == 0.95)

    def test_invalid_arguments(self):
        for kw in ({"count": -1}, {"intensity": 1.5}, {"time_range_s": (50.0, 10.0)}):
            with self.assertRaises(ValueError):
                randomized_ignitions(self.scn, 0, **kw)


class OutcomeTests(unittest.TestCase):
    def facts(self, **kw):
        base = dict(goals_total=5, goals_reached=5, base_reached=True, ended_at_refuge=False, aborted=False,
                    fault_latched=False, timed_out=False)
        base.update(kw)
        return RunFacts(**base)

    def test_taxonomy(self):
        self.assertIs(classify_outcome(self.facts()), Outcome.COMPLETE)
        self.assertIs(classify_outcome(self.facts(goals_reached=4)), Outcome.PARTIAL)
        self.assertIs(classify_outcome(self.facts(base_reached=False, ended_at_refuge=True)), Outcome.REFUGE)
        self.assertIs(classify_outcome(self.facts(base_reached=False, aborted=True)), Outcome.ABORTED)
        self.assertIs(classify_outcome(self.facts(base_reached=False, fault_latched=True)), Outcome.STUCK)
        self.assertIs(classify_outcome(self.facts(base_reached=False, timed_out=True)), Outcome.TIMEOUT)
        with self.assertRaises(ValueError):
            classify_outcome(self.facts(goals_reached=6))


class StatisticsTests(unittest.TestCase):
    def test_wilson_known_values(self):
        lo, hi = wilson_interval(10, 10)
        self.assertAlmostEqual(lo, 0.7225, places=3)
        self.assertEqual(hi, 1.0)
        lo, hi = wilson_interval(0, 10)
        self.assertEqual(lo, 0.0)
        self.assertAlmostEqual(hi, 0.2775, places=3)
        lo, hi = wilson_interval(15, 30)
        self.assertAlmostEqual((lo + hi) / 2, 0.5, places=2)
        self.assertEqual(wilson_interval(0, 0), (0.0, 1.0))
        with self.assertRaises(ValueError):
            wilson_interval(5, 3)

    def test_mean_sd(self):
        m, sd = mean_sd([2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0])
        self.assertAlmostEqual(m, 5.0)
        self.assertAlmostEqual(sd, 2.13809, places=4)
        self.assertEqual(mean_sd([3.0]), (3.0, 0.0))
        with self.assertRaises(ValueError):
            mean_sd([])

    def test_paired_bootstrap_is_deterministic_and_covers_the_mean(self):
        a, b = [5.0, 6.0, 7.0, 8.0, 9.0, 10.0], [1.0, 2.0, 3.5, 3.0, 5.0, 4.0]
        r1, r2 = paired_bootstrap_ci(a, b, iterations=2000), paired_bootstrap_ci(a, b, iterations=2000)
        self.assertEqual(r1, r2)
        mean, lo, hi = r1
        self.assertAlmostEqual(mean, sum(x - y for x, y in zip(a, b)) / 6)
        self.assertLess(lo, mean)
        self.assertGreater(hi, mean)
        self.assertGreater(lo, 0.0)  # every pair differs by > 2
        with self.assertRaises(ValueError):
            paired_bootstrap_ci([1.0], [1.0, 2.0])


if __name__ == "__main__":
    unittest.main()
