import json
import tempfile
import unittest
from pathlib import Path

from aegisnav_core.cli import run_demo


class DemoTests(unittest.TestCase):
    def test_demo_emits_reviewable_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            summary = run_demo(Path(tmp))
            self.assertTrue(Path(tmp, "metrics.json").exists())
            self.assertTrue(Path(tmp, "aegisnav_route_comparison.svg").exists())
            metrics = json.loads(Path(tmp, "metrics.json").read_text())
            self.assertEqual(len(metrics), 2)
            self.assertEqual(summary["scenario_id"], "corridor_hazard_spread_v2")
            self.assertIn("risk_aware_safe_return_success", summary)
            self.assertEqual(summary["risk_aware_safe_return_success"], summary["risk_aware"]["safe_return_success"])
            self.assertEqual(summary["baseline_safe_return_success"], summary["baseline"]["safe_return_success"])


if __name__ == "__main__":
    unittest.main()
