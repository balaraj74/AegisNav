# ADR-0009: Batch study methodology (full Gazebo/Nav2 missions)

- **Status:** Accepted, 2026-11-17. Pilot verified (one complete `rescue_compound` sensing mission, 601 s wall-clock). Results: `docs/simulation/batch_results.md`.
- **Context:** The only evidence so far was single demonstration runs and a 20-seed kinematic study on the small corridor map. A reviewer cannot tell a lucky run from a reliable system. The question to answer is how often the integrated stack (Gazebo, Nav2, AMCL, AegisNav) completes the mission, and what it costs in simulated hazard exposure.

## Decision

1. **Unit of evidence: one complete headless mission** per (mode, seed) in its own `--network none` container. Gazebo, Nav2 and the AegisNav nodes run exactly as in the demo. Display-only nodes (console, Gazebo hazard tiles) are off.
2. **Scenario variation:** `rescue_compound` with its four permanent sources plus three **seeded random ignitions** (`aegisnav_core.batch.randomized_ignitions`). Cells are free, more than 8 cells from the start and more than 3 from any goal; times are uniform in 20–240 s of mission time. The same seed gives identical ignitions in every mode.
3. **Four configurations on identical seeds**
   | Mode | Hazard knowledge | Planner / policy |
   |---|---|---|
   | `sensing` | onboard sensor only | full AegisNav + triage |
   | `sensing_notriage` | onboard sensor only | full AegisNav, fixed point order |
   | `omniscient` | the real field | full AegisNav + triage |
   | `baseline` | the real field | hazard-blind shortest path (`baseline_distance_only`: hazard and uncertainty weight 0, hard limit 0.99, no mission policy) |
4. **Measured against the real field, not the robot's belief.** `mission_recorder_node` samples the true hazard layer (`/aegisnav/hazard_truth` with sensing, else the forecast layer 0) at the robot's AMCL position every 0.5 s of simulated time. The position estimate was measured against Gazebo ground truth at 0.05–0.36 m error (mean 0.13 m); that is a stated limitation, not hidden. Exposure is simulated-model risk × seconds, not physical dose.
5. **Outcome taxonomy** (`aegisnav_core.batch.classify_outcome`, tested): `complete` (all five points and base), `partial` (base reached, a point never reached), `refuge`, `aborted`, `stuck` (no progress for 120 s), `timeout` (900 s mission clock). A run without a finished result counts as a failure, never as missing data.
6. **Scope actually run (2026-11-17).** The plan was 15 seeds × 4 modes. At ~12 minutes per batch of three runs that was ~3.5 hours, so it was cut for time: `sensing` and `baseline` (the headline comparison) were run on matched seeds 1–8; `omniscient` and `sensing_notriage` only on the few seeds that finished before the cut. The report marks those two as too small to compare.
7. **Statistics:** rates with 95% Wilson intervals; costs as mean ± SD; paired comparisons on shared seeds with a percentile bootstrap (10 000 resamples, fixed rng). No significance tests are claimed; with 8 runs per mode the intervals are wide and the report says so.
8. **Parallelism:** three containers at once (3 CPUs each, 4 GB). Wall-clock planner cycle times are therefore measured under load and are reported as such. Mission outcomes use the simulated clock.

## Consequences

- The baseline in Gazebo is the same planner with hazard terms removed, not a different navigation stack. It answers "does hazard-awareness help", not "is Nav2's own planner better".
- Hysteresis is not ablated in the batch: in this scenario the computed confidence stays near 0.8, so it rarely latches. It is covered by unit tests only.
- The hazard field, the sensor and the confidence estimate are synthetic. Results describe this simulator.
- Per-run artifacts (`metrics.json`, filtered `sim.log`, `exit_reason.txt`) are committed under `artifacts/batch/`; `tools/analyze_batch.py` regenerates the report from them.
