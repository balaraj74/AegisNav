# Closed-loop evaluation (kinematic abstraction)

Run: `./tools/run_eval.sh 20 0` (about 2-3 minutes). Output: `artifacts/eval/results.json`, `summary.svg`, and a seed-0 trace/map. A short illustrative run where the ignition changes the decision: `artifacts/showcase_seed7/`.

## What this is
`aegisnav_core.simulation` runs hazard truth -> belief forecast -> mission policy -> `ExecutionController` -> simulated action server -> point-robot motion at 1 s ticks. Both modes see the **same** map, start, goal, refuges, ground-truth hazard and events per seed. The baseline is a one-shot distance-only A* (hazard and uncertainty weights 0, block threshold 0.99) with no mission policy and no replanning. It is not a Nav2 baseline and not a strong baseline.

## What this is not
No Gazebo, Nav2, localization, sensor, controller or collision behaviour. The hazard is the synthetic diffusion model. The scenario distributions (ignition cell/time, confidence drop, speed) are design choices of this study.

## Metric definitions
See `metric_definitions` in `results.json` (single source: `experiment.py`).

## Results, 20 seeds (0-19), three modes, run 2026-10-30

Supersedes the 2026-10-29 table below. It was re-run after exact travel-time accounting (ADR-0008) and with the new onboard-sensing mode. The seeds, map, start, goal, refuges and ground-truth hazard are the same for all modes.
- **Omniscient:** the belief is forecast from all sources known at time t.
- **Onboard sensing:** the robot knows only what its simulated 4 m line-of-sight sensor has seen.

| | AegisNav omniscient | AegisNav onboard sensing | Distance baseline |
|---|---|---|---|
| Reached target | 17/20 | 16/20 | 20/20 |
| Completed and returned to base | 7/20 | 7/20 | 20/20 |
| Ended at a refuge | 13/20 | 13/20 | 0 |
| Time at truth risk >= 0.92 (s), mean±sd [min,max] | 0.0±0.0 [0,0] | 0.0±0.0 [0,0] | 1.65±0.49 [1,2] |
| Hazard exposure (risk·s) | 5.27±4.00 [0.35,14.55] | 5.49±4.01 [0.43,14.24] | 14.12±3.58 [8.16,20.76] |
| Replans (replacement dispatches) | 0.80±0.70 | 1.05±0.89 | 0 |
| Path length (m) | 17.7±3.1 | 18.1±2.9 | 19.3 |
| Mission time (s) | 55.4±8.7 | 57.1±8.4 | 59.0±4.7 |

Reading:
- In this synthetic setup, the sensing robot's exposure is close to the omniscient planner's.
- It replans slightly more often, because it learns of hazards only when it sees them.
- Neither AegisNav mode entered a cell at or above the hard threshold; the baseline did in every trial.
- The corridor scenario is small: most hazards come into the 4 m sensor range early. A larger area (`rescue_compound`) would test discovery harder and has not been evaluated yet.
- Twenty trials of one scenario family is not strong evidence, and no significance test was run. Do not quote a percentage improvement.

## Results, 20 seeds (0-19), map v2 (after the one-cell-passage fix), run 2026-10-29 (superseded)

Supersedes the earlier v1 table (one-cell gates; see ADR-0005). Same seeds, map, start, goal, refuges and ground-truth hazard for both modes.

| | AegisNav | Distance baseline |
|---|---|---|
| Reached target | 17/20 (0.85) | 20/20 |
| Completed and returned to base | 7/20 | 20/20 |
| Ended at a refuge | 13/20 | 0 |
| Time at truth risk >= 0.92 (s), mean±sd [min,max] | 0.0±0.0 [0,0] | 1.65±0.49 [1,2] |
| Time at truth risk >= 0.5 (s) | 3.8±3.8 | 9.1±3.4 |
| Hazard exposure (risk·s) | 5.6±4.2 [0.4,14.5] | 14.1±3.6 [8.2,20.8] |
| Replans (replacement dispatches) | 0.8±0.7 | 0 |
| Path length (m) | 17.5±3.0 | 19.3 |
| Mission time (s) | 55.0±8.4 | 59.0±4.7 |
| Planning time per cycle (ms, this host) | 161±40 | 12±1 |
| Aborts | 0 | 0 |

Reading: AegisNav never entered a cell at or above the hard threshold and had lower exposure in this synthetic setup, at the cost of finishing the full mission in 7/20 trials (13 ended at a refuge, mostly after the simulated confidence drop, which occurs in about half the seeds). The baseline always finished and always entered hazard, mostly at the permanent lower-gate fire. Twenty trials of one scenario family is not strong evidence; no significance test was run; do not quote a percentage improvement.

## Gazebo cross-check (not a statistical result)

Seed 7 run live (headless Gazebo + Nav2 in the Jazzy container) showed the same decision sequence as the kinematic runner: `continue_to_target` → cancel + replacement route at the 16 s ignition → `slow_and_reassess` at the 28 s confidence drop → arrival at refuge (21,2). See `docs/testing/verification_matrix.md` for the exact runs.
