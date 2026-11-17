import random
import unittest

from aegisnav_core.forecast import HazardForecaster
from aegisnav_core.models import Cell, ForecastConfig, GridMap
from aegisnav_core.sensing import HazardBelief, detected, estimate_confidence, observe, visible_cells


def walled(width=9, height=5, wall_x=4, gap_y=None):
    g = GridMap(width, height)
    return GridMap(width, height, 1.0, frozenset(g.index(Cell(wall_x, y)) for y in range(height) if y != gap_y))


class VisibilityTests(unittest.TestCase):
    def test_wall_occludes_and_is_itself_visible(self):
        g = walled()
        vis = visible_cells(g, Cell(1, 2), 10.0)
        self.assertIn(g.index(Cell(3, 2)), vis)
        self.assertIn(g.index(Cell(4, 2)), vis)  # the wall face
        self.assertNotIn(g.index(Cell(6, 2)), vis)  # behind the wall

    def test_gap_gives_line_of_sight(self):
        g = walled(gap_y=2)
        self.assertIn(g.index(Cell(7, 2)), visible_cells(g, Cell(1, 2), 10.0))

    def test_range_limit(self):
        g = GridMap(20, 3)
        vis = visible_cells(g, Cell(0, 1), 4.0)
        self.assertIn(g.index(Cell(4, 1)), vis)
        self.assertNotIn(g.index(Cell(5, 1)), vis)

    def test_no_sight_between_diagonal_blocks(self):
        g0 = GridMap(3, 3)
        g = GridMap(3, 3, 1.0, frozenset({g0.index(Cell(1, 0)), g0.index(Cell(0, 1))}))
        self.assertNotIn(g.index(Cell(1, 1)), visible_cells(g, Cell(0, 0), 5.0))

    def test_smoke_hides_what_is_behind_it(self):
        g = GridMap(9, 1)
        smoke = frozenset({g.index(Cell(4, 0))})
        vis = visible_cells(g, Cell(0, 0), 10.0, obscured=smoke)
        self.assertIn(g.index(Cell(4, 0)), vis)  # the smoke itself is seen
        self.assertNotIn(g.index(Cell(5, 0)), vis)
        self.assertIn(g.index(Cell(5, 0)), visible_cells(g, Cell(4, 0), 10.0, obscured=smoke))  # robot inside it

    def test_invalid(self):
        g = GridMap(3, 3)
        with self.assertRaises(ValueError):
            visible_cells(g, Cell(5, 5), 2.0)
        with self.assertRaises(ValueError):
            visible_cells(g, Cell(0, 0), float("nan"))


class ObservationTests(unittest.TestCase):
    def test_noise_is_seeded_and_clamped(self):
        truth = [0.0, 0.5, 1.0]
        a = observe(truth, {0, 1, 2}, noise_sigma=0.2, rng=random.Random(3))
        b = observe(truth, [2, 1, 0], noise_sigma=0.2, rng=random.Random(3))
        self.assertEqual(a, b)  # independent of input order
        self.assertTrue(all(0.0 <= v <= 1.0 for v in a.values()))
        self.assertEqual(observe(truth, {1}), {1: 0.5})
        with self.assertRaises(ValueError):
            observe(truth, {1}, noise_sigma=0.1)


class BeliefTests(unittest.TestCase):
    def test_unknown_is_uncertain_not_safe(self):
        g = GridMap(4, 1)
        belief = HazardBelief(g)
        risk, unc = belief.layers(0.0)
        self.assertEqual(risk, (0.0,) * 4)
        self.assertEqual(unc, (0.6,) * 4)

    def test_discovery_is_a_new_region_not_spread(self):
        g = GridMap(12, 1)
        belief = HazardBelief(g, region_radius_cells=2)
        self.assertEqual(belief.update({1: 0.9, 2: 0.8}, 0.0), [Cell(1, 0)])  # one region, reported once
        self.assertEqual(belief.update({3: 0.7}, 1.0), [])  # spread of the known fire
        self.assertEqual(belief.update({9: 0.9}, 2.0), [Cell(9, 0)])  # a separate fire
        belief.update({9: 0.45}, 3.0)  # noisy reading dips below the threshold...
        self.assertEqual(belief.update({9: 0.55}, 4.0), [])  # ...and back: the same fire, not a new one
        with self.assertRaises(ValueError):
            HazardBelief(g, region_radius_cells=-1)

    def test_discovery_sources_and_aging(self):
        g = GridMap(4, 1)
        belief = HazardBelief(g, source_threshold=0.5)
        self.assertEqual(belief.update({1: 0.9, 2: 0.2}, 10.0), [Cell(1, 0)])
        self.assertEqual(belief.update({1: 0.95}, 11.0), [])  # already known: not a new discovery
        self.assertEqual([s.cell for s in belief.sources()], [Cell(1, 0)])
        risk, unc = belief.layers(21.0)
        self.assertEqual(risk[2], 0.2)
        self.assertAlmostEqual(unc[2], 0.1 + 0.01 * 11.0)
        self.assertEqual(unc[0], 0.6)
        belief.update({1: 0.1}, 30.0)  # fire seen out: no longer a source
        self.assertEqual(belief.sources(), ())

    def test_blocked_and_invalid_readings(self):
        g0 = GridMap(3, 1)
        g = GridMap(3, 1, 1.0, frozenset({g0.index(Cell(1, 0))}))
        belief = HazardBelief(g)
        belief.update({1: 0.9}, 0.0)
        self.assertEqual(belief.observed(), frozenset())
        with self.assertRaises(ValueError):
            belief.update({0: 1.5}, 0.0)
        with self.assertRaises(ValueError):
            HazardBelief(g, source_threshold=0.0)

    def test_belief_forecast_spreads_only_what_was_seen(self):
        g = GridMap(9, 1)
        belief = HazardBelief(g)
        belief.update({2: 0.9}, 0.0)  # a fire at (8, 0) exists but was never seen
        risk0, unc0 = belief.layers(0.0)
        frames = HazardForecaster().forecast(g, risk0, unc0, belief.sources(), ForecastConfig(horizon_steps=3)).frames
        self.assertGreater(frames[3].at(Cell(3, 0))[0], 0.0)
        self.assertEqual(frames[3].at(Cell(8, 0))[0], 0.0)

    def test_detected_targets(self):
        g = walled()
        vis = visible_cells(g, Cell(1, 2), 10.0)
        self.assertEqual(detected(vis, g, [Cell(6, 2), Cell(3, 1)]), [Cell(3, 1)])


class ConfidenceTests(unittest.TestCase):
    def test_view_and_localization_limit_confidence(self):
        self.assertEqual(estimate_confidence(100, 100, 0.0), 1.0)
        self.assertAlmostEqual(estimate_confidence(40, 100, 0.0), 0.4)
        self.assertAlmostEqual(estimate_confidence(100, 100, 0.35, max_localization_std_m=0.5), 0.3)
        self.assertAlmostEqual(estimate_confidence(100, 100, 0.15), 0.85)  # normal AMCL tracking
        self.assertLess(estimate_confidence(100, 100, 0.46), 0.55)        # ~one 0.5 m cell of doubt: degraded
        self.assertLessEqual(estimate_confidence(100, 100, 0.7), 0.30 + 1e-9)  # critical
        self.assertEqual(estimate_confidence(100, 100, 2.0), 0.0)

    def test_invalid(self):
        for args in ((5, 0, 0.1), (11, 10, 0.1), (-1, 10, 0.1), (5, 10, float("nan")), (5, 10, -0.1)):
            with self.assertRaises(ValueError):
                estimate_confidence(*args)


if __name__ == "__main__":
    unittest.main()
