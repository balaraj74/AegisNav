import unittest

from aegisnav_core.forecast import HazardForecaster
from aegisnav_core.models import Cell, ForecastConfig, GridMap, HazardSource


class HazardForecastTests(unittest.TestCase):
    def test_risk_spreads_to_adjacent_free_cell(self):
        grid = GridMap(5, 3)
        risk = [0.0] * grid.size
        risk[grid.index(Cell(1, 1))] = 1.0
        result = HazardForecaster().forecast(
            grid, risk, sources=[HazardSource(Cell(1, 1), 1.0)],
            config=ForecastConfig(horizon_steps=3, spread_rate=0.3, decay_rate=0.0),
        )
        self.assertGreater(result.frames[1].at(Cell(2, 1))[0], 0.0)
        self.assertEqual(len(result.frames), 4)

    def test_source_intensity_is_present_in_current_forecast_layer(self):
        grid = GridMap(3, 3)
        result = HazardForecaster().forecast(
            grid, [0.0] * grid.size,
            sources=[HazardSource(Cell(1, 1), 0.8)],
            config=ForecastConfig(horizon_steps=1),
        )
        self.assertAlmostEqual(result.frames[0].at(Cell(1, 1))[0], 0.8)

    def test_blocked_wall_prevents_direct_spread(self):
        blocked = frozenset(y * 5 + 2 for y in range(3))
        grid = GridMap(5, 3, blocked=blocked)
        risk = [0.0] * grid.size
        risk[grid.index(Cell(1, 1))] = 1.0
        result = HazardForecaster().forecast(
            grid, risk, sources=[HazardSource(Cell(1, 1), 1.0)],
            config=ForecastConfig(horizon_steps=5, spread_rate=0.5, decay_rate=0.0),
        )
        self.assertAlmostEqual(result.frames[-1].at(Cell(3, 1))[0], 0.0)

    def test_invalid_risk_values_fail_fast(self):
        with self.assertRaises(ValueError):
            HazardForecaster().forecast(GridMap(2, 2), [0.0, 0.0, 0.0, 2.0])


if __name__ == "__main__":
    unittest.main()
