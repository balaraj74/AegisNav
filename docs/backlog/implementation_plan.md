# Implementation plan and acceptance criteria

Priority order is designed around a short hackathon deadline; complete vertical slices rather than creating a broad but disconnected feature set.

## P0 — Core algorithm and trustworthy evidence

- [x] Normalized hazard and uncertainty data contracts.
- [x] Deterministic forecast layers with blocked-cell handling.
- [x] Time-indexed risk-aware A* with a hard cutoff.
- [x] Return-route planning at target arrival time.
- [x] Mission policy for confidence degradation, return, refuge and abort.
- [x] Baseline-vs-candidate demo with JSON/SVG artifacts.
- [x] Unit tests for core failure cases.

**Acceptance:** `./tools/run_core_tests.sh`, `./tools/run_demo.sh`, and `./tools/verify_repo.sh` pass; metrics label the synthetic model.

## P1 — ROS interfaces and static map bringup

- [x] `HazardForecast.msg` interface source.
- [x] ROS forecast node reading a traversability map.
- [x] ROS route/decision adapter.
- [x] RViz marker overlay and aligned sample map/world source.
- [ ] Build all packages on a real Jazzy/Harmonic host.
- [ ] Confirm map/image orientation, source cells, forecast layer dimensions and TF/frame naming in Gazebo/RViz.

**Acceptance:** clean colcon build and documented smoke-test evidence.

## P2 — Real closed-loop Nav2 integration

- [ ] Bring up localization and Nav2 controller server for the chosen robot.
- [ ] Verify `FollowPath` action contract, controller/goal checker IDs and frame IDs.
- [ ] Implement automatic local replanning trigger with a defined hysteresis/debounce policy.
- [ ] Demonstrate abort/no-route cancels active goals and the robot stops.
- [ ] Confirm local costmap collision avoidance remains authoritative over global route proposals.

**Acceptance:** repeated scene runs, no stale action execution after abort, and recorded trace.

## P3 — Stronger research contribution

- [ ] Add reactive-current-layer baseline (B1).
- [ ] Add multiple hazard-spread rates and source locations.
- [ ] Introduce reproducible scenario seeds and 20+ trials per configuration.
- [ ] Report mean/dispersion for exposure, completion, return success, path length and planning runtime.
- [ ] Search and compare prior research before asserting novelty.
- [ ] Consider a physically appropriate hazard simulator only if it materially strengthens the research question.

**Acceptance:** reproducible results with scripts, environment versions and explicit limitations.

## P4 — Presentation quality

- [ ] Integrate an appropriately licensed reusable disaster scene.
- [ ] Align collision/map geometry and include model license/attribution.
- [ ] Add a clean RViz side-panel/decision timeline.
- [ ] Test camera framing, video capture load and offline fallback on the competition laptop.

## Implementation log 2026-10-29
Done: execution state machine; closed-loop seeded runner; 20-trial study; stale-input handling; ADR-0004; source review.
Known gaps: no ROS/Gazebo run (host is Ubuntu 26.04); `hazard_forecast_node` has no ignition/confidence scenario source, so the ROS stack cannot yet reproduce the seeded dynamic events; no AMCL/Nav2 launch; Nav2 controller/goal-checker IDs and action name unverified; no 3D asset upgrade; planning time ~0.2 s per cycle on this host.
Next highest-value task: on Ubuntu 24.04 + Jazzy, build the workspace and add a scenario node that publishes ignition and confidence events from `DynamicScenario`, then smoke-test FollowPath accept/cancel/replace.

## Update (same day, later)
Added: scenario node, ignition input, TF pose source, localization/controller params, full launch, asset-consistency tests (map/SDF/spawn verified by source test). Still unrun on any ROS machine. Remaining: execute smoke test stages 0-3 and fix what fails; confirm lidar frame id, bridge remaps, DWB params; RViz decision overlay and mission status panel; screenshot/recording from the running sim; asset upgrade only after the core demo runs.

## Update 2026-10-29 (live integration)
Done: Jazzy container; colcon build/test; stage 1; full Gazebo + AMCL + RPP + AegisNav loop; seed 7 (reroute + degraded-confidence refuge) and seed 3 (target and return) driven to completion; eight integration defects found and fixed (ADR-0005).
Known gaps: no GPU/RViz/visual run on the target laptop; no recording; 20-trial study only in the kinematic runner; one transient FollowPath failure near base in seed 3; hazard visuals in the SDF are static and not driven by the forecast; no 3D asset upgrade.
Next highest-value task: on the RTX 2050 laptop, run `sim_world.launch.py seed:=7 start_rviz:=true` with the GUI, confirm RTF/visuals, and record the demo.

## Update 2026-10-29 (interactive testing)
User confirmed on the laptop GUI (sandbox mode, Intel GPU) that fires placed in RViz cause a cancel and a detour, and that lowering confidence sends the robot to a refuge. Next highest-value task: record the 90-110 s demo from `tools/run_gui.sh sandbox` (or `demo 7`) following `docs/operations/demo_runbook.md`; optionally install the NVIDIA Container Toolkit for the RTX 2050.
