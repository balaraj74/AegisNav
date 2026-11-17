import unittest

from aegisnav_ros.director import Director, Shot


class DirectorTests(unittest.TestCase):
    def test_sequence(self):
        d = Director(overview_s=6, cutaway_s=5)
        self.assertEqual(d.shot(0).kind, "overview")
        self.assertEqual(d.shot(7).kind, "chase")
        d.on_fire(10, 1.0, 2.0)
        self.assertEqual(d.shot(12), Shot("fire", (1.0, 2.0)))
        d.on_decision_change(12)  # does not interrupt the fire cut-away
        self.assertEqual(d.shot(14).kind, "fire")
        self.assertEqual(d.shot(15.5).kind, "chase")
        d.on_decision_change(20)
        self.assertEqual(d.shot(21).kind, "tactical")
        self.assertEqual(d.shot(26).kind, "chase")

    def test_simultaneous_fires_keep_one_cutaway(self):
        d = Director(overview_s=0, cutaway_s=5)
        d.on_fire(10, 1.0, 2.0)
        d.on_fire(10.5, 1.5, 2.0)
        self.assertEqual(d.shot(14), Shot("fire", (1.0, 2.0)))
        self.assertEqual(d.shot(15.5).kind, "chase")
        d.on_fire(20, 3.0, 3.0)  # a later fire gets its own cut-away
        self.assertEqual(d.shot(21), Shot("fire", (3.0, 3.0)))

    def test_invalid(self):
        with self.assertRaises(ValueError):
            Director(cutaway_s=0)


if __name__ == "__main__":
    unittest.main()
