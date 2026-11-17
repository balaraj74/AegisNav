# Verification matrix

## Status when this starter was prepared

| Area | State | Evidence / next check |
|---|---|---|
| Pure-Python hazard forecast | Implemented | Unit tests cover diffusion to free neighbors, wall blocking and invalid risk values |
| Time-indexed A* | Implemented | Unit tests cover route found, critical-cell exclusion, blocked goals and diagonal corner cutting |
| Mission policy | Implemented | Tests cover feasible mission, degraded/critical confidence and missing forecast |
| Deterministic demo | Implemented | Test verifies SVG/JSON artifact generation; inspect output before publishing numbers |
| Root instruction set | Implemented | `AGENTS.md`, `.agent/rules`, roles and workflows present |
| ROS message source | Implemented, generation unverified | Build with `colcon` on a Jazzy machine to validate generated interfaces |
| ROS grid/message adapters | Source-only contract tests | Fake-message tests validate dimensions, frame ids, axis-aligned geometry and forecast encoding; live ROS message integration remains unverified |
| ROS hazard node | Implemented, not target-smoke-tested | Confirm map callback, source-cell validation, forecast dimensions, marker publication; map geometry changes reset the synthetic risk field |
| ROS planner node | Implemented, not target-smoke-tested | Confirm conversion, pose/goal contract, route topic, JSON trace and abort path clearing |
| Gazebo SDF | Authored, not simulator-validated here | Load with `gz sim`; inspect physics, robot spawn, lidar, DiffDrive command and scan/odom topic bridges |
| Static map alignment | **Verified by source test** (`aegisnav_bringup/test/test_assets.py`): map yaml origin/resolution, PGM obstacles (y-flip), every SDF wall box vs every grid cell, robot spawn cell = start cell | Visual check in RViz/Gazebo still pending |
| Full bringup launch (`sim_world.launch.py`: bridges+remaps, static TF, AMCL, controller_server, lifecycle) | Written, `py_compile` only | `docs/simulation/ros_smoke_test.md` stage 3 |
| Planner pose from TF (`pose_source: tf`) | Written, `py_compile` only | Verify staleness behaviour while stationary |
| Execution state machine (`execution.py`) | Implemented, unit-tested (no ROS) | `test_execution.py`: single goal in flight, cancel-before-replace, pending dropped on stop, late-accepted goal, fault latch, stale inputs |
| Closed-loop kinematic simulation | Implemented, tested | `test_simulation.py`: seed replay identical, ignition changes decision, robot static between cancel and next acceptance. Abstraction only; not Gazebo |
| Baseline vs AegisNav study | 20 seeded trials run | `artifacts/eval/results.json`; see `docs/simulation/closed_loop_evaluation.md` |
| Scenario node + ignition input (`scenario_node.py`, `timeline.py`) | Timeline unit-tested; nodes `py_compile` only | Run `docs/simulation/ros_smoke_test.md` stage 1 |
| `planner_node` ↔ ExecutionController wiring | Written, `py_compile` only | Needs Jazzy build + Nav2 smoke test (goal accept/cancel/replace, stale inputs, server missing) |
| Nav2 FollowPath action | Opt-in integration, unverified | Keep disabled until controller server, local costmap, frames and cancel/replan are tested |
| Photorealism | Not provided by primitive scene | Reuse a licensed high-quality disaster world and verify it on the target GPU |
| Physical fire/gas model | Not implemented | Current model is synthetic; do not claim physical validity |
| Real perception confidence | Not implemented | Current confidence is an external test signal |

## Required release smoke test

Record environment and commit hash, then run:

1. `./tools/run_core_tests.sh` (89 core + 18 ROS adapter + 12 bringup asset tests as of 2026-10-29)
2. `./tools/run_demo.sh`
3. `./tools/verify_repo.sh`
4. `colcon build --symlink-install` on Ubuntu 24.04 / ROS 2 Jazzy.
5. Launch Gazebo world and verify `/clock`, scan, odometry, map, forecast and markers.
6. Send known start/goal messages and verify the path's first/last cell.
7. Change confidence and hazard source configuration, verify reason-code transitions.
8. If action execution is enabled, verify a changed route cancels the old goal and abort/no-route cancels movement.
9. Record CPU/GPU/RAM use and confirm the launch survives several repeated runs without stale ROS processes.

No integration status should be upgraded from “unverified” based only on a successful Python syntax check.

## Host record 2026-10-29

Ubuntu 26.04.1, Python 3.14.4, no native ROS. Steps 4–9 were executed in the `aegisnav-jazzy` Docker image (Ubuntu 24.04 / Jazzy / Harmonic), headless; see the live-results table below. Not executed: GPU rendering on the target laptop, RViz, video capture.

## Live ROS 2 / Gazebo results (2026-10-29, `aegisnav-jazzy` container)

Environment: Docker image from `tools/docker/Dockerfile.jazzy` (`ros:jazzy` = Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic via `ros-jazzy-ros-gz`, Nav2 1.3.x). Gazebo ran **headless** (`-s --headless-rendering`) with software rendering (no GPU in the container); RTF and visuals on the RTX 2050 laptop were **not** measured. Each run isolated with `--network none`.

| Check | Result | Evidence |
|---|---|---|
| `rosdep install`, `colcon build --symlink-install` | Pass (4 packages) after fixing missing `aegisnav_ros/__init__.py` and an invalid `ament_python` rosdep key | `tools/docker/run_in_jazzy.sh` |
| `colcon test` / `colcon test-result` | Pass: 107 tests, 0 failures (core 89 + ros adapter 18) on final code; bringup asset tests (12) run via `tools/run_core_tests.sh` | after adding `tests_require` to `aegisnav_ros` |
| Stage 1: nodes + map server, fake pose, no Gazebo | Pass on final code: `continue_to_target` → `slow_and_reassess` at the 28 s confidence drop (on map v2 the ignition changes the route, not the action) | `tools/docker/stage1.sh` |
| Gazebo world loads; `/clock`, `/scan`, `/odom`, TF | Pass; scan frame `aegis_bot/base_link/front_lidar` | `tools/docker/stage3.sh` |
| Lidar vs map geometry (robot static at spawn) | Pass: max abs error 0.02 m over 12 bearings | `tools/docker/check_lidar.py` |
| Robot kinematics | Pass after ADR-0005 fixes: straight forward motion, pitch 0.00° under step accel/brake | drive/brake probes |
| AMCL accuracy vs Gazebo ground truth | 0.07–0.11 m with the 0.05 m map (was 0.31 m on the 0.4 m grid) | stage-3 ground-truth block |
| `map_server`, `planning_map_server`, `amcl`, `controller_server` lifecycle | active | `ros2 lifecycle get` |
| `/follow_path` action | present, `nav2_msgs/action/FollowPath`, goals accepted | `ros2 action list -t` |
| Dynamic hazard changes executed behaviour (seed 7) | Pass: ignition at 16 s → cancel → replacement route; confidence drop at 28 s → `slow_and_reassess` → cancel → route to refuge → "Reached the goal" at refuge (truth 0.21 m from cell centre) | seed-7 final run log |
| Full mission incl. return (seed 3) | Pass: target reached → goal switched to base → base reached (one transient FollowPath failure, then success) | seed-3 final run log |
| Cancel stops the robot | Controller logs "Cancellation was successful. Stopping the robot." | seed-7 runs |
| Abort sends no new goal | Observed in an earlier (pre-fix) run: `no_safe_route` → cancel, no further dispatch; not re-run after final fixes | run 3 log |
| Fault latch after 3 controller failures | Observed: dispatch stops after three `failed` results | runs 4–13 |
| Repeated full runs | Two consecutive final runs (seeds 3, 7) passed; earlier runs exposed and fixed the defects in ADR-0005 | — |
| RViz visuals, GPU rendering, video | **Not run** (headless container) | — |
| 20-trial evaluation **in Gazebo** | **Not run**; the 20-trial study is the kinematic runner | `artifacts/eval/results.json` |

### Visualization checks (2026-10-29)
| Check | Result | Evidence |
|---|---|---|
| Lidar vs map after scene changes | max abs error 0.02 m | `tools/docker/check_lidar.py` |
| Hazard visuals invisible to lidar | PASS (flagged box unseen, unflagged seen at 0.43 m) | `tools/docker/check_visibility_mask.py` |
| Gazebo hazard tiles follow forecast | Bands grow over the run (e.g. band 3: 1 → 31 cells); first per-tile design stalled on a lost reply and was replaced | visualization node log |
| Seed 7 completes with visualization running | Yes (ignition → reroute, confidence drop → refuge reached) | capture run events |
| Stale-pose fail-closed observed live | Yes: under extra software-render load the TF went stale > 3 s; planner cancelled and published no path, then resumed and reached the refuge | render run 1 |
| Rendered frames | Genuine headless captures (ogre2/llvmpipe, Xvfb for RViz) in `docs/assets/screenshots/` | `tools/docker/capture_demo.sh` |
| GPU frame rate / GUI on RTX 2050 | **Not measured** | — |
| Sandbox: user fire on active route | Cancel + replacement route; old route passed the fire cell, new route does not; target still reached | `tools/docker/sandbox_test.sh` |
| Sandbox: all openings on fire | `seek_refuge (target_route_unavailable)`, refuge reached | `tools/docker/sandbox_block_test.sh` |
| Sandbox: confidence 0.2 | `seek_refuge (perception_confidence_critical)`, refuge reached | `tools/docker/sandbox_test.sh` |
| GUI on the laptop | User ran `run_gui.sh demo`: robot moved; feedback "looked hardcoded" led to sandbox mode | user report |
| Sandbox with GUI on the laptop (Intel Iris Xe via `/dev/dri`) | User reported the fire-reroute and confidence tests behaved as described on the test card (2026-10-29). First sandbox build waited for a goal without saying so; fixed: robot now drives to the target by default | user report |

## Rescue-building world, console and director (2026-10-29, `aegisnav-jazzy` container)

Rendering: Intel iGPU via `/dev/dri` (Mesa iris), headless sensors; GUI runs on Xvfb. Laptop display frame rate is **not** measured.

| Check | Result | Evidence |
|---|---|---|
| Unit tests | 104 core + 35 ROS adapter + 13 bringup asset, all pass; `colcon test` 139/139 in the container | `tools/run_core_tests.sh`, `colcon test-result` |
| World/map/scenario alignment for every preset | Pass (walls vs grid, spawn = start cell, fine map, wheel joints, level robot) | `aegisnav_bringup/test/test_assets.py` |
| Asset licenses | 14 Fuel models, CC0 per Fuel metadata, SHA-256 pinned | `tools/fuel_assets.lock.json`, `THIRD_PARTY_NOTICES.md` |
| Husky diff drive + AMCL | Rotates in place; AMCL error 0.03 m vs ground truth | drive probe, stage-3 ground truth |
| Scripted mission, final script (headless) | Ignitions at 9 s (scheduled 8 s) → cancel + replacement route at 12 s; victim 1 ~58 s → victim 2 ~97 s → return; confidence 0.42 → `slow_and_reassess (perception_confidence_degraded)` at cell (28, 4) → cancel → refuge reached | full-mission run log |
| Earlier script (ignition at 22 s, Xvfb GUI + headless) | Ignition was off-route: no reroute; confidence drop came after the robot was home. Led to the re-timed script and `test_scripted_fire_blocks_first_leg_and_a_detour_exists` | run logs |
| Director with GUI | Overview → chase → fire cut-away (flames centred) → chase, confirmed in 1 s frames. A single `gz topic` publish was once lost and left the camera on the fire shot; each shot is now sent three times over 1 s, and chase resumed in the re-run | `tools/docker/gui_capture.sh` frames |
| Fire placed from the console on the active route | Cancel → replacement route around the fire; shortest-path comparison flagged `crosses_hard_limit` | headless run + console in the browser pane |
| Recording layout | Gazebo GUI shows only the 3D view + play/stats overlays | `gui_capture.sh` frames |
| Source markers invisible to lidar | PASS: forward range 3.33 m empty, 3.33 m with fire marker, 0.43 m with an unflagged box | `tools/docker/check_visibility_mask.py` |
| Particle effects (2026-10-29) | **Removed**: rendered as opaque squares on Mesa/ogre2 | GUI frames |
| Planner cycle time | ~1.1–1.3 s per cycle while continuing; 1.5–2.3 s after the confidence drop (refuge assessment, spread hazard); warning logged above 1 s | planner log |
| Confidence slider / critical confidence in this world | **Not run in this world** (verified in the corridor world) | — |
| 20-trial statistics in this world | **Not run** | — |

## Fire and smoke particle effects (2026-10-30, `aegisnav-jazzy` container, Mesa iris, GUI on Xvfb)

| Check | Result | Evidence |
|---|---|---|
| Fuel `Fog Emitter` v5 | CC BY 4.0 per Fuel metadata; SHA-256 pinned; attribution recorded | `tools/fuel_assets.lock.json`, `THIRD_PARTY_NOTICES.md` |
| Emitter with `fog.png` + colour-range image renders | Soft puffs, not squares. Colours applied only with absolute texture paths (`model://` colour image "not found" → grey) | spike GUI frames |
| Effect spawned by `effect_model_sdf` in `rescue_building` | Glowing flame cones with orange particle tips and a grey smoke column; green gas cloud over floor haze | dedicated GUI capture (camera at the director's fire-shot pose) |
| Full launch with GUI (`gui_capture.sh`) | Both source effects spawned without errors; mission reached waypoint 2 and returned to base; `slow_and_reassess` on the confidence drop | `sim.log`, 16 frames |
| Effects for the scripted ignitions | **Not spawned** in that run: only the two scenario sources got effects (existing visualization behaviour, not changed here) | `sim.log` |
| `particle_scatter_ratio` 0 hides particles from the lidar | **No**: forward range 3.33 m → 0.34–0.80 m (0.0001: 0.19–1.34 m) with particles 0.04–0.09 m above the scan plane | scratch control runs |
| Effects invisible to lidar, corridor world (plugin injected) | PASS: 3.33 m empty, 3.33 m with the fire effect, 0.43 m with an unflagged box; gas 3.33 m over 10 scans | `tools/docker/check_visibility_mask.py` |
| Effects invisible to lidar, `rescue_building` (Husky, 0.55 m) | PASS: fire and gas 0.6 m and 0.3 m ahead, 10 scans each, 4.55–4.56 m = empty | scratch control script (forward ±3 beams only) |
| Unit tests | 125 core + 39 ROS adapter + 13 bringup asset, all pass (counts include another session's concurrent test changes) | `tools/run_core_tests.sh` |
| Native NVIDIA rendering, laptop GUI | **Not run** | — |

## Sprint 2026-10-30: sensing, larger area, speed (container, headless)

| Check | Result | Evidence |
|---|---|---|
| Unit tests | 126 core + 40 ROS adapter + 16 bringup asset, all pass | `tools/run_core_tests.sh` |
| Speed-ups do not change results | A* + `evaluate_route` (360 queries), forecaster (12 forecasts), forecast-message conversion: bit-identical to the 2026-10-29 code. The cost-to-go heuristic: same success and optimal cost on 156 queries, with distance-only (baseline) plans identical in every field (90 queries) | `~/aegisnav_work/equiv*.py` (outside the repo); `test_heuristic_is_admissible_and_consistent` |
| Planner cycle, rescue_building (live) | p95 0.58 s, median 0.24 s (was p95 1.04 s, median 0.82 s) | 107 logged cycles |
| Planner cycle, rescue_compound + sensing (live) | 0 cycles over 1 s in a full mission (previous run: 140) | sim log |
| Sensing mission, rescue_building | Gas found at 15 s and fire at 19 s, reroute at 22 s; both victims sensed and reached; base reached | sim logs |
| Sensing + triage mission, rescue_compound | All 5 inspection points, 4 hazard regions discovered and avoided, 4 victims sensed; base at 429 s after first pose; AMCL error vs Gazebo truth max 0.36 m, mean 0.13 m; computed confidence min 0.81; RTF ~0.68 | sim log + `loc_probe.py` |
| Earlier compound failure | Robot pinned by a misplaced pallet stack; odometry/AMCL drifted 7 m from truth. Fixed (ADR-0007 amendment); new mesh-footprint tests | probe CSV |
| Particle fire/smoke vs lidar | Particles start ≥ 1.0 m (Husky lidar 0.55 m); full sensing mission without phantom obstacles | separate session's tests; sim log |
| Three-mode kinematic study (20 seeds, corridor) | omniscient 5.27±4.00, sensing 5.49±4.01, baseline 14.12±3.58 risk·s; no AegisNav entry at risk ≥ 0.92 | `docs/simulation/closed_loop_evaluation.md` |
| **Not verified** | Any of this on the laptop GUI; triage set-aside/retry live; smoke-driven confidence drop live; a compound study across many seeds | — |

## Submission video (2026-11-02)

`artifacts/video/AegisNav_demo.mp4` (2:21, 1080p, silent): one complete `rescue_compound` sensing + triage mission recorded in the container on virtual displays (software-rendered Gazebo GUI at ~0.4 real-time factor, 15 fps capture). Result of that run: 5/5 inspection points, 4 hazard regions discovered, 4 victims found, base reached, no stale-pose stops, no controller failures. Sped up ×2 to ×15, labelled on screen. Captions were generated from that run's log. One run is an illustration, not statistical evidence.

## Batch study (2026-11-17, `aegisnav-jazzy` container, headless, 3 runs in parallel)

| Check | Result | Evidence |
|---|---|---|
| Full missions, AegisNav sensing + triage, 6 seeds | 6/6 complete (all 5 points, back at base); 0/6 entered risk ≥ 0.92; exposure 6.7 ± 2.7 risk·s | `docs/simulation/batch_results.md` |
| Full missions, shortest-path baseline, same 6 seeds | 5/6 complete; 6/6 entered risk ≥ 0.92 (1.5–2.0 s each); exposure 16.8 ± 6.2; 1 stuck (controller "collision ahead" near courtyard goal) | same |
| Paired exposure difference, AegisNav − baseline | −10.1 risk·s, 95% bootstrap CI −15.0 to −4.7 (lower in 5 of 6 seeds; seed 5's baseline stopped early) | same, `docs/simulation/batch_findings.md` |
| Stale-pose stops | 0 in all runs | run logs |
| Omniscient and no-triage configurations | 2 seeds each only: **too few to compare** | same |
| Not done | reactive (current-layer) baseline; more than 6 seeds; hysteresis ablation; runs on the laptop GPU | ADR-0009 |
