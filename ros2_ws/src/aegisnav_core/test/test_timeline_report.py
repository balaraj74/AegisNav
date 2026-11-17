import tempfile
import unittest
from pathlib import Path

from aegisnav_core.timeline_report import action_segments, render_timeline_svg

TRACE = [
    {"t": 0, "event": "decision", "action": "continue_to_target", "reason_code": "mission_feasible"},
    {"t": 5, "event": "decision", "action": "continue_to_target", "reason_code": "mission_feasible"},
    {"t": 10, "event": "hazard_ignition"},
    {"t": 10, "event": "decision", "action": "seek_refuge", "reason_code": "target_route_unavailable"},
    {"t": 10, "event": "cancel", "reason_code": "replan_cancel"},
    {"t": 20, "event": "terminal", "outcome": "reached_refuge"},
]


class TimelineReportTests(unittest.TestCase):
    def test_identical_consecutive_decisions_merge(self):
        self.assertEqual(action_segments(TRACE, 20), [
            (0, 10, "continue_to_target", "mission_feasible"),
            (10, 20, "seek_refuge", "target_route_unavailable"),
        ])

    def test_svg_written_with_reason_codes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp, "t.svg")
            render_timeline_svg(path, TRACE, "t")
            text = path.read_text()
        self.assertIn("target_route_unavailable", text)
        self.assertIn("ignition t=10", text)

    def test_empty_trace_rejected(self):
        with self.assertRaises(ValueError):
            render_timeline_svg("x.svg", [], "t")


if __name__ == "__main__":
    unittest.main()
