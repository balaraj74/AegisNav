# Changelog

## 0.1.0 — Starter scaffold

- Added a ROS-independent deterministic hazard propagation, time-indexed A* planner, mission safety supervisor, and evaluation CLI.
- Added a synthetic 24×16 rescue scenario and distance-only baseline comparison.
- Added ROS 2 message and adapter sources, a Gazebo Harmonic sample world, static map, topic bridges, and RViz configuration.
- Added agent contract (`AGENTS.md`, `agent.md`, `.agent/`), contribution workflow, CI source-quality checks, ADRs, and research-claim discipline.
- Added source-only regression tests for core planning and ROS grid/message contracts.

### Verification boundary

The local source-only tests and structural checks pass. The ROS 2 workspace has not been compiled on a real Jazzy/Harmonic installation, and the Gazebo world has not been launched in this preparation environment. FollowPath execution remains disabled by default. See `docs/testing/verification_matrix.md`.

## 2026-10-29
- Added `execution.py` (dispatch/cancel state machine, stale-input check), `MissionAssessment.execution_route`, `simulation.py` (seeded closed-loop runner), `experiment.py` (20-trial study, JSON + SVG), `tools/run_eval.sh`, ADR-0004, `docs/research/source_review.md`.
- `planner_node` now delegates dispatch to `ExecutionController`, fails closed on stale pose/forecast; only `py_compile`d, not run under ROS.
- Core tests: 65 (was 15), plus 14 ROS adapter contract tests.
- Added `scenario_node`, ignition input on the forecast node, TF-based planner pose, Nav2 localization/local-costmap params, full `sim_world.launch.py`, `aegisnav_bringup/test/test_assets.py`.
- Fixed config defect found by the new test: DWB max speed was 0.18 m/s while the planner assumed 0.35 m/s; controller now 0.35.
- Added `timeline_report.py` (decision-timeline SVG) and `tools/docker/Dockerfile.jazzy` (24.04/Jazzy test image).
- Live integration in the `aegisnav-jazzy` container (ADR-0005): map v2 with 0.8 m openings and generated assets; fixed wheel joint axes, caster geometry (front+rear), localization map split (0.05 m `/map`, 0.4 m `/aegisnav/planning_map`), Regulated Pure Pursuit controller, path centring/densifying, pose snapping, hysteresis, wall-clearance penalty, TF freshness, map QoS, sim-time launch arg, package `__init__`/rosdep/test fixes, scenario return phase and hold-at-destination.
- Tests: 89 core + 18 ROS adapter + 12 bringup asset; `colcon test` 107/107 in Jazzy.
- `test_simulation`: the seed-7 assertion "ignition changes the action" was replaced by "ignition triggers cancel + replacement route" and "confidence drop changes the action", because on map v2 the ignition reroutes within CONTINUE (verified identical in Gazebo).
- Visuals (ADR-0006): forecast-driven Gazebo hazard tiles (one model per band, lost-reply tolerant), lidar visibility mask, removal of static hazard models that the lidar saw as obstacles, RViz current vs predicted hazard + mission/decision overlay + footprint, generated pads/threshold bars/exterior rubble, lower interior walls, restyled robot, fill lights. Map fix: removed an unintended one-cell passage at (10,7) and added a test forbidding one-cell passages. Genuine renders in `docs/assets/screenshots/`.
- Interactive testing: sandbox mode (`tools/run_gui.sh sandbox`), RViz 2D Goal Pose / Publish Point (click = fire), `tools/sim_cmd.sh`, `/aegisnav/execution_event`, on-screen event log + decision explanation + grey ghost of the cancelled route, test card `docs/operations/try_it.md`. Verified headless: a fire placed on the active route caused cancel + a replacement route that avoids the fire cell; blocking all openings gave `seek_refuge (target_route_unavailable)`.
- Rescue-building mission (ADR-0007):
  - **Scenario:** a 40×28 core scenario at 0.5 m, with two victims, three refuges, fire and gas sources, a scripted ignition and a confidence drop. The world and maps are generated from the scenario grid; the asset tests check every preset in `config/worlds.yaml`.
  - **Assets:** 14 pinned, license-checked CC0 Fuel models (`tools/fetch_fuel_assets.py`, `THIRD_PARTY_NOTICES.md`).
  - **Robot:** a Husky with a true differential-drive model.
  - **Planner rules:** hard obstacle inflation, plus a no-diagonal-near-walls rule shared by the planner and `evaluate_route`.
  - **Mission sequencing:** multi-goal sequencing (`next_goal`, `goal_cell` tagging) and `/aegisnav/mission_status`.
- **Operator console** (`aegisnav-console`, http://localhost:8080):
  - live map, forecast, route, cancelled-route ghost, decision explanation, metrics;
  - shortest-path comparison computed by the planner on the same forecast;
  - simulated-input controls for fire, goal and confidence.
- **Recording:**
  - event-driven Gazebo camera director (`aegisnav-director`);
  - recording GUI layout without editor panels (`config/gazebo_gui_recording.config`);
  - hazard-source markers as translucent, lidar-invisible geometry. Particle emitters were tried and removed because they rendered as opaque squares.
- **Demo script fix:** the original scripted ignition (22 s) was off the robot's route and the confidence drop came after the mission ended. Both were re-timed and placed using the core planner, and a regression test now guards the script.
- **Tests:** 104 core, 35 ROS adapter and 13 bringup asset tests; `colcon test` 139/139.

## 2026-10-30
- **Hazard source effects (ADR-0007 item 8, revised):**
  - Fire is now a particle flame and smoke column over a glowing ember bed and translucent flame cones; gas is a hovering green particle cloud over a floor haze. These are visual effects driven by the scenario's source cells, not combustion or dispersion physics.
  - The `gz-sim-particle-emitter-system` plugin is back in the generated `rescue_building` world.
  - Particles reuse the CC BY 4.0 Fuel `Fog Emitter` v5 texture. The fire, smoke and gas colour ramps are generated by `tools/generate_world_assets.py`.
- **Lidar finding:** `particle_scatter_ratio` 0 does not hide particles from the `gpu_lidar`. Particles are therefore kept 0.45 m above the highest lidar plane, enforced by a test. `check_visibility_mask.py` passes; fire and gas 0.3 m and 0.6 m ahead of the Husky leave its scan unchanged.
- `tools/fetch_fuel_assets.py`:
  - allows CC0 and CC BY 4.0 and records an attribution line for CC BY;
  - adds `--only NAME`.
- Tests (all pass): 125 core + 39 ROS adapter + 13 bringup asset, counts including concurrent changes from another session. Not run: native NVIDIA rendering, laptop GUI, `colcon test`.


## 2026-10-30 — Sprint: the robot discovers hazards, larger inspection area, faster planning
- **Planner speed (no change in results):**
  - the A* search runs on integer cell indices with precomputed neighbour tables and a single-layer fast path;
  - the forecaster uses neighbour tables;
  - the forecast-message conversion is table-driven;
  - the hazard-blind comparison route is cached per (start, target).

  All were proven bit-identical against the previous code. Live p95 planning cycle in the container: 1.04 s → 0.58 s (median 0.82 → 0.24 s).
- **Exact travel-time accounting** (ADR-0008, intentionally changes results): moves are no longer rounded up to whole forecast layers, which had overestimated travel time about 2.4× with 2 s layers.
- **Onboard sensing** (`sensing:=true`, ADR-0008):
  - `aegisnav_core.sensing`: line of sight, observation, `HazardBelief`, discovery and smoke occlusion;
  - `hazard_sensor_node`;
  - a forecaster `belief` mode;
  - a hidden `/aegisnav/hazard_truth`;
  - `FIRE DETECTED` and `VICTIM FOUND` console events with a "Truth" layer;
  - director cut-aways on discovery;
  - perception confidence computed from view and localization quality.

  The kinematic study gains a sensing mode. Verified in full headless Gazebo/Nav2 missions in `rescue_building`.
- **Tactics:**
  - confidence hysteresis (`resume_confidence` 0.65);
  - triage of inspection points by lowest-exposure safe route, with set-aside and retry (`triage:=true`).
- **`rescue_compound` world:**
  - 36 × 22 m: the building plus a courtyard, a yard and a warehouse, with 5 inspection points;
  - generated from the core scenario;
  - exterior models are placed relative to the grid edges; the `rescue_building` world is byte-identical apart from its header comment;
  - a new asset test checks every generated world's maps and walls against its scenario.
- **Realistic fire and smoke** (separate session): CC BY 4.0 Fuel fog texture particles kept above 1.0 m (lidar at 0.55 m), with geometry below. See ADR-0007.
- `.claude/settings.local.json` (per-user tool settings) is git-ignored and skipped by `verify_repo`.
