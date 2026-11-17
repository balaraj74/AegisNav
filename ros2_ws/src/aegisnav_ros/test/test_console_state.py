import unittest

from aegisnav_ros.console_state import MissionMetrics, parse_confidence, parse_point


class MetricsTests(unittest.TestCase):
    def test_integrates_distance_exposure_and_hazard_time(self):
        m = MissionMetrics(near_risk=0.5, hard_risk=0.9)
        m.update(0.0, 0.0, 0.0, 0.0)
        m.update(1.0, 3.0, 4.0, 0.6)
        m.update(2.0, 3.0, 4.0, 0.95)
        d = m.as_dict()
        self.assertEqual(d["distance_m"], 5.0)
        self.assertAlmostEqual(d["exposure_risk_s"], 1.55)
        self.assertEqual((d["time_near_s"], d["time_in_s"]), (2.0, 1.0))
        self.assertEqual(d["peak_risk"], 0.95)

    def test_out_of_order_samples_ignored(self):
        m = MissionMetrics()
        m.update(5.0, 0.0, 0.0, 0.0)
        m.update(4.0, 10.0, 0.0, 1.0)
        self.assertEqual(m.distance_m, 0.0)

    def test_invalid(self):
        with self.assertRaises(ValueError):
            MissionMetrics(near_risk=0.9, hard_risk=0.5)
        with self.assertRaises(ValueError):
            MissionMetrics().update(float("nan"), 0, 0, 0)


class ParseTests(unittest.TestCase):
    def test_point_and_confidence(self):
        self.assertEqual(parse_point({"x": "1.5", "y": -2}), (1.5, -2.0))
        self.assertEqual(parse_confidence({"value": 0.3}), 0.3)
        for bad in ({"x": "inf", "y": 0}, {"y": 0}):
            with self.assertRaises((ValueError, KeyError)):
                parse_point(bad)
        with self.assertRaises(ValueError):
            parse_confidence({"value": 1.2})


if __name__ == "__main__":
    unittest.main()
