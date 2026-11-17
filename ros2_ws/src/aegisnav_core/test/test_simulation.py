import unittest
from dataclasses import replace

from aegisnav_core.models import Cell, HazardSource
from aegisnav_core.simulation import (
    DynamicScenario, Mode, Outcome, TruthField, run_trial, seeded_dynamic_scenario,
)

SEED = 7  # short AegisNav run (ends ~23 s); keeps the suite fast


class ScenarioValidationTests(unittest.TestCase):
    def test_seed_replay_gives_identical_scenario(self):
        self.assertEqual(seeded_dynamic_scenario(3), seeded_dynamic_scenario(3))
        self.assertNotEqual(seeded_dynamic_scenario(3), seeded_dynamic_scenario(4))

    def test_invalid_parameters_rejected(self):
        ds = seeded_dynamic_scenario(0)
        for change in ({"ignition_time_s": 0}, {"confidence_after": 1.5}, {"robot_speed_mps": 0.0},
                       {"reassess_period_s": 0}, {"near_risk": -0.1}):
            with self.assertRaises(ValueError):
                replace(ds, **change)

    def test_ignition_in_wall_rejected(self):
        ds = seeded_dynamic_scenario(0)
        with self.assertRaises(ValueError):
            replace(ds, ignition_sources=(HazardSource(Cell(0, 0), 1.0),))


class ClosedLoopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ds = seeded_dynamic_scenario(SEED)
        cls.truth = TruthField(cls.ds)
        cls.aegis = run_trial(cls.ds, cls.truth, Mode.AEGISNAV)
        cls.base = run_trial(cls.ds, cls.truth, Mode.BASELINE)

    def test_ignition_changes_truth_field(self):
        cell = self.ds.ignition_sources[0].cell
        k = self.ds.ignition_time_s
        self.assertLess(self.truth.risk_at(k - 1, cell), 0.5)
        self.assertGreaterEqual(self.truth.risk_at(k, cell), self.ds.ignition_sources[0].intensity)

    def test_replay_same_seed_is_identical(self):
        again = run_trial(self.ds, self.truth, Mode.AEGISNAV)
        self.assertEqual(self.aegis.deterministic_signature(), again.deterministic_signature())

    def test_every_decision_event_has_reason_code_and_explanation(self):
        decisions = [e for e in self.aegis.trace if e["event"] == "decision"]
        self.assertTrue(decisions)
        for event in decisions:
            self.assertTrue(event["reason_code"] and event["explanation"])

    def test_ignition_triggers_cancel_and_replacement_route(self):
        # Map v2 (two-cell gates): the ignition blocks one gate cell, so the policy keeps
        # CONTINUE but reroutes through the other one at the ignition tick.
        k = self.ds.ignition_time_s
        at_k = [e["event"] for e in self.aegis.trace if e["t"] == k]
        self.assertIn("cancel", at_k)
        replacement = [e for e in self.aegis.trace if e["t"] == k and e["event"] == "send"]
        self.assertEqual(replacement[0]["reason_code"], "replacement_dispatch")
        self.assertNotIn(self.ds.ignition_sources[0].cell, self.aegis.trajectory)

    def test_confidence_drop_changes_the_action(self):
        k = self.ds.confidence_drop_time_s
        before = [e for e in self.aegis.trace if e["event"] == "decision" and e["t"] < k]
        after = [e for e in self.aegis.trace if e["event"] == "decision" and e["t"] >= k]
        self.assertEqual(before[-1]["action"], "continue_to_target")
        self.assertEqual(after[0]["action"], "slow_and_reassess")
        self.assertEqual(after[0]["reason_code"], "perception_confidence_degraded")

    def test_robot_never_moves_between_cancel_and_next_acceptance(self):
        trace, traj = self.aegis.trace, self.aegis.trajectory
        for i, event in enumerate(trace):
            if event["event"] != "cancel":
                continue
            resume = next((e["t"] for e in trace[i:] if e["event"] == "goal_response"), len(traj))
            window = traj[event["t"]: resume + 1]
            self.assertEqual(set(window), {window[0]}, f"robot moved after cancel at t={event['t']}")

    def test_at_most_one_goal_in_flight(self):
        in_flight = 0
        for event in self.aegis.trace + []:
            if event["event"] == "send":
                in_flight += 1
            elif event["event"] == "result":
                in_flight -= 1
            self.assertLessEqual(in_flight, 1)

    def test_baseline_uses_identical_truth_and_never_replans(self):
        self.assertEqual(self.base.replans, 0)
        self.assertEqual(self.base.seed, self.aegis.seed)
        self.assertEqual(self.base.outcome, Outcome.COMPLETED_AND_RETURNED)

    def test_aegisnav_avoids_hazard_the_baseline_enters(self):
        self.assertEqual(self.aegis.time_in_hazard_s, 0)
        self.assertGreater(self.base.time_in_hazard_s, 0)
        self.assertLess(self.aegis.hazard_exposure, self.base.hazard_exposure)

    def test_sensing_robot_learns_hazards_only_by_seeing_them(self):
        sensing = run_trial(self.ds, self.truth, Mode.AEGISNAV_SENSING)
        found = [e for e in sensing.trace if e["event"] == "hazard_discovered"]
        ignition = self.ds.ignition_sources[0].cell
        seen_ignition = [e["t"] for e in found if any(abs(x - ignition.x) <= 2 and abs(y - ignition.y) <= 2
                                                       for x, y in e["cells"])]
        self.assertTrue(seen_ignition, "the ignition region is never discovered")
        self.assertGreaterEqual(seen_ignition[0], self.ds.ignition_time_s)  # no prescience
        cancels = [e["t"] for e in sensing.trace if e["event"] == "cancel"]
        self.assertTrue(any(t >= seen_ignition[0] for t in cancels), "discovery did not change the route")
        self.assertLess(sensing.max_risk_encountered, 0.92)
        self.assertEqual(run_trial(self.ds, self.truth, Mode.AEGISNAV_SENSING).deterministic_signature(),
                         sensing.deterministic_signature())

    def test_metrics_dict_is_json_serialisable(self):
        import json
        json.dumps(self.aegis.metrics_dict())


if __name__ == "__main__":
    unittest.main()
