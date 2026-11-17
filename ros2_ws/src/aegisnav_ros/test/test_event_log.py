import unittest

from aegisnav_ros.event_log import EventLog, wrap


class EventLogTests(unittest.TestCase):
    def test_bounded_and_ordered(self):
        log = EventLog(max_lines=2)
        for i in range(3):
            log.add(float(i), f"e{i}")
        self.assertEqual(log.lines(), ["t=  1.0s  e1", "t=  2.0s  e2"])

    def test_dedupe_by_key(self):
        log = EventLog()
        self.assertTrue(log.add(1.0, "fire at (1,2)", key="fire:1:2"))
        self.assertFalse(log.add(2.0, "fire at (1,2)", key="fire:1:2"))
        self.assertEqual(len(log.lines()), 1)

    def test_invalid(self):
        with self.assertRaises(ValueError):
            EventLog(max_lines=0)

    def test_wrap_truncates(self):
        out = wrap("word " * 50, width=20, max_lines=2)
        self.assertEqual(len(out.split("\n")), 2)
        self.assertTrue(out.endswith("..."))
        self.assertEqual(wrap("short"), "short")


if __name__ == "__main__":
    unittest.main()
