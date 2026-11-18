# Batch study: findings and limits (current planner, with the fire margin)

Numbers: `docs/simulation/batch_results.md` (regenerate with `tools/analyze_batch.py`). Method: ADR-0009. Why the margin exists: ADR-0010. The earlier study of the planner **without** the margin is kept as `batch_findings_v1_no_standoff.md` and is not comparable.

## What was run

Full Gazebo/Nav2 missions in `rescue_compound`, one container per run, three seeded random fires per seed on top of four permanent sources. AegisNav (onboard sensing, triage, 1.5 m fire margin) against a hazard-blind shortest-path baseline, on matched seeds. **Only seeds 1 and 2 finished** for both modes: the margin and the longer forecast roughly doubled planning time, runs took 20-30 minutes each, and the study was stopped for lack of time. Two matched runs are illustrative, not evidence.

## What the two runs show

| | AegisNav seed 1 | AegisNav seed 2 | Baseline seed 1 | Baseline seed 2 |
|---|---|---|---|---|
| Completed all 5 points, back at base | yes | yes | yes | yes |
| Closest approach to a burning cell | 2.49 m | 1.04 m | 0.14 m | 0.15 m |
| Time at risk >= 0.92 | 0 s | 0 s | 2.0 s | 1.5 s |
| Hazard exposure, risk.s | 17.2 | 10.8 | 25.7 | 16.9 |
| Mission time, simulated s | 459 | 321 | 319 | 325 |
| Route changes | 10 | 8 | 1 | 0 |

- The robot with the margin stayed at least 1.04 m from any burning cell; the baseline drove through them (0.14-0.15 m).
- **The margin has a cost.** AegisNav's missions got longer (seed 1: 459 s against 289 s without the margin) and its exposure higher (seed 1: 17.2 against 6.2 without the margin), because it detours and spends more time in the faint outer halo of fires (risk around 0.1-0.2). It still had lower exposure than the baseline on both seeds (by 8.5 and 6.1 risk.s), but the gap is smaller than the 10.1 risk.s found without the margin.
- Seed 2's closest approach of 1.04 m is just outside the 1.0 m limit the margin was built around. The robot is about 1 m long, so on screen that is still close.

## What the earlier (no-margin) study showed, and why it was superseded

Six matched seeds without the margin: AegisNav 6/6 complete and 0/6 at risk >= 0.92, baseline 5/6 complete and 6/6 at risk >= 0.92; mean exposure 6.7 against 16.8 risk.s. Those numbers were measured correctly but hid a problem: in the AegisNav runs that recorded positions the robot passed 0.60-0.66 m from a burning cell, so it looked like it drove over the fire. The margin (ADR-0010) was added because of that.

## Limits

- Two matched runs per mode. No intervals or significance are claimed.
- The baseline is a hazard-blind shortest path, not Nav2's own planner and not a reactive planner using only the current hazard layer.
- Everything is synthetic: the hazard model, the sensor, the confidence estimate.
- The margin (1.5 m centre to burning cell) is a simulation rule, not a validated safety distance.
- Exposure is measured at the AMCL pose (within 0.05-0.36 m of Gazebo truth in an earlier check).
- Runs were executed several at a time on a loaded machine; wall-clock planner cycle times are not clean.

## Reproduce

```bash
MAX_WALL_S=3000 tools/run_batch.sh "sensing baseline" 1 4 4   # seeds 3-4 were not completed
PYTHONPATH=ros2_ws/src/aegisnav_core python3 tools/analyze_batch.py
```
