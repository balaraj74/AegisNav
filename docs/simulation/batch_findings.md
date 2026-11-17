# Batch study: findings and limits (2026-11-17)

Source numbers: `docs/simulation/batch_results.md` (regenerate with `tools/analyze_batch.py`). Method: ADR-0009. This is a small simulation study; read the limits at the end first.

## What was run

- Full Gazebo/Nav2 missions in `rescue_compound` (36 × 22 m, five inspection points), one container per run, seeded random ignitions (3 per seed) on top of four permanent hazard sources.
- **Headline comparison, 6 matched seeds each:** AegisNav with onboard sensing and triage, against a hazard-blind shortest-path baseline (same planner with hazard and uncertainty weights 0, no mission policy).
- Two further configurations (knows every fire; no triage) ran on only 2 seeds before the batch was stopped for time. They are in the report but are **too few to compare**.

## What was found (6 matched seeds)

| | AegisNav, sensing + triage | Shortest-path baseline |
|---|---|---|
| Completed all 5 points and returned to base | 6/6 (95% CI 61–100%) | 5/6 (95% CI 44–97%) |
| Runs that entered risk ≥ 0.92 | 0/6 (95% CI 0–39%) | 6/6 (95% CI 61–100%) |
| Time at risk ≥ 0.92 per run | 0.0 s | 1.5–2.0 s |
| Hazard exposure, risk·s (mean ± SD) | 6.7 ± 2.7 | 16.8 ± 6.2 |
| Mission time, simulated s | 295 ± 6 | 313 ± 19 |
| Distance, m | 127 ± 3 | 123 ± 29 |

- **Exposure:** on identical seeds AegisNav's exposure was lower by 10.1 risk·s on average (95% bootstrap CI −15.0 to −4.7). It was lower in five of the six seeds (by 5.1 to 18.6 risk·s). In seed 5 the baseline's exposure was *lower* (7.1 against 8.3), but that run stopped after 3 of 5 points, so it did less of the mission; excluding that seed the mean difference is −12.3 risk·s.
- **Deadly-zone entries:** the baseline touched a cell at risk ≥ 0.92 in every run, briefly (1.5–2.0 s, i.e. cutting through the edge of a hazard region). AegisNav did not in any run, while discovering the hazards with its own 4 m line-of-sight sensor.
- **Cost of caution:** AegisNav was not slower or longer in these runs (295 s vs 313 s; 127 m vs 123 m), so avoiding hazard did not cost mission time here.
- **Reliability of the integrated stack:** all six AegisNav sensing runs and five of six baseline runs completed. There were no stale-pose stops in any run. One AegisNav run (seed 2) had a single transient controller failure ("collision ahead" 11 times, one failed `FollowPath`) and recovered by replanning; no AegisNav run latched the fault. The slowest planner cycle was 1.8 s under three parallel simulations.

## The one failure, and what it shows

Baseline seed 5 stopped after three points. About two cells short of inspection point 5 (courtyard) the local controller reported "collision ahead" 27 times and "patience exceeded" three times, which latched the execution fault. This is a controller/world interaction, not a hazard effect: point 5 sits diagonally next to the rubble corner added so the courtyard victim fits the grid, and the baseline probably approached it from a different direction than the AegisNav runs (not verified). AegisNav could hit the same spot on another seed. The follow-up is to move that goal away from the rubble; the world was deliberately **not** changed mid-study, so the runs stay comparable.

## Limits (all of these apply)

- **Six runs per mode.** The intervals are wide; no significance test is claimed and no percentage improvement is quoted.
- **The comparison is against a hazard-blind shortest path**, not against Nav2's own global planner and not against a reactive planner that uses only the current hazard layer. The latter is the fairest test of whether *forecasting* helps and has not been run.
- **Everything is synthetic:** the hazard model (diffusion-like rule, not fire physics), the sensor, and the confidence estimate.
- **Position estimate:** exposure is measured at the AMCL pose (earlier measured within 0.05–0.36 m of Gazebo truth, mean 0.13 m).
- **Load:** runs were executed three at a time while other programs were using the machine. Mission outcomes use the simulated clock; wall-clock planner cycle times in the report were measured under that load.
- **Scope cut for time:** the plan was 15 seeds × 4 configurations; 6 seeds × 2 configurations (plus 2 seeds of two more) were run.

## Reproduce

```bash
tools/run_batch.sh "sensing baseline" 1 6 3      # about 10-20 min per run, three at a time
PYTHONPATH=ros2_ws/src/aegisnav_core python3 tools/analyze_batch.py
```
