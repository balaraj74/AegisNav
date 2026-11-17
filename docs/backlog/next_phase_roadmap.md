# Next phase roadmap (drafted 2026-11-17)

Goal: make AegisNav measurably more credible, not just more impressive-looking. Each phase ends with evidence a skeptical reviewer can reproduce. Priorities follow AGENTS.md: no claim without a measurement.

## What "perfect" means here
"Perfect working" cannot be proven. What can be shown is a **measured reliability number**: N seeded full-mission runs in Gazebo, with success rate, failure causes and a confidence interval. Phase 1 produces it.

## Phase 0 — Make the work safe and reproducible (1 day) — DONE 2026-11-17
Committed locally in pieces and tagged `v0.2-sprint-2026-10`; console wording, fire-truck textures and ROS build/test CI done. Not pushed anywhere.
- **First commit.** The repo has no commits yet; all sprint work exists only in the working tree. Commit in logical pieces (core, ROS, bringup/assets, docs), tag `v0.2-sprint-2026-10`.
- **Fix known rough edges:**
  - console banner says "heading to the victim" for points without a victim;
  - fire-truck textures missing (`suv.png`);
  - (checked) `SetColorRange is currently disabled` only logs a warning: the recorded flames render orange and the gas green.
- **CI:** GitHub Actions running `run_core_tests.sh`, `verify_repo.sh`, and `colcon build/test` in the `aegisnav-jazzy` image.

## Phase 1 — Prove reliability and the research claim (3–4 days) — PARTLY DONE 2026-11-17
Done: batch runner, truth-based recorder, outcome taxonomy, statistics, 6 matched seeds sensing vs baseline (`docs/simulation/batch_findings.md`). Not done: 30+ runs, ablations beyond 2 seeds, stress cases, reactive baseline. Found: the courtyard goal sits next to rubble (controller failure); move it.
1. **Headless batch runner:** `tools/run_batch.sh N` runs N compound missions with seeds, scripted fires randomized per seed, and records per-run outcome, time, exposure, planner cycle times and failure reason.
2. **Report:** 30+ runs × {omniscient, sensing, distance-only baseline}; mean ± SD, success rate with a Wilson 95% interval, plus the failure taxonomy (stuck, stale pose, timeout, hazard entry).
3. **Ablations** (each switch off in turn): sensing, triage, confidence hysteresis, exact-time accounting. Shows which feature earns its place.
4. **Stress cases:** dynamic fire blocking a doorway, two simultaneous ignitions, sensor noise ×3, localization degradation. Report where it breaks, not only where it works.
- **Done when:** one table in `docs/simulation/` with the above, reproducible from one command.

## Phase 2 — Real training: a learned perception module (1–2 weeks)
This is the honest "real training" item, because it replaces a stub with something measurable.

- **Today:** victim detection is grid line of sight; confidence is a hand-made formula (documented as not calibrated).
- **Build:**
  - add an RGB camera to the Husky (Gazebo `camera` sensor);
  - generate a labeled dataset in Gazebo by placing the robot at many poses (teleport, not drive) with domain randomization (lighting, victim pose, smoke, props), capturing victim / fire / smoke labels from ground truth;
  - train a small CNN (MobileNet-class) with PyTorch; hold out entire rooms/scenarios for testing, not random frames;
  - measure precision/recall and **calibration** (expected calibration error, reliability diagram);
  - a new `aegisnav_perception` package consumes images and publishes detections and a calibrated confidence on the existing topics. `aegisnav_core` stays stdlib-only; the model never enters the core.
- **Constraints:** RTX 2050 with 4 GB; PyTorch is installed CPU-only here. A small model trains on CPU in hours for this data size, and the GPU build can be used if you install it. Needs an ADR (new dependency, torch is BSD-3) and a license check on any pretrained weights.
- **Honest claim afterwards:** "the detector achieves X precision/recall on held-out simulated scenes, ECE Y". Not a claim about real-world perception.
- **Done when:** the sensing mission runs with the learned detector instead of line of sight, and the Phase 1 batch is re-run with it.

## Phase 3 — Smarter decisions (1 week, optional)
- **Learned triage ordering:** train a small model to predict leg exposure and time from features, compare against the current planner-based ranking on the seeded study. Keep it only if it measurably wins; otherwise report that it doesn't.
- **Parameter tuning (not RL):** tune hazard weight, uncertainty weight and thresholds with Bayesian optimization on the kinematic simulator, evaluated on held-out seeds.
- Reinforcement learning is **not** recommended: it is hard to beat a time-indexed A* here and the simulator is not rich enough to make the result meaningful.

## Phase 4 — More realism (parallel, 1–2 weeks)
- **Hazards:** door/ceiling collapse that really blocks routes (fused from the local costmap), wind-driven gas drift in the scenario, smoke that degrades the camera image.
- **Sensing:** thermal camera model (Gazebo thermal) as a second hazard sensor.
- **Scenario:** second compound layout and randomized building generator for generalization tests.
- **Rendering:** install the NVIDIA Container Toolkit (a system change you make) to use the RTX 2050 and drop software-rendering limits; re-record.

## Phase 5 — Presentation that holds up (3–4 days)
- Narrated 2–3 minute video with the batch results shown, not one lucky run.
- Short technical report (6–8 pages): problem, method, ablations, limits, reproducibility.
- Mission report generator: after each run, one HTML/PDF with timeline, decisions with reason codes, exposure, and the route-vs-shortest-path comparison.
- Live demo script with a tested fallback (kept: building demo).

## Recommended order
Phase 0 → Phase 1 → Phase 2 → Phase 5; Phase 4 in parallel; Phase 3 only if time remains.

## Things I would not do
- An LLM "planner", cloud services or Isaac Sim: they add risk without answering the research question.
- Claims of "real-time", "safe" or "AI-powered" without Phase 1/2 numbers behind them.
