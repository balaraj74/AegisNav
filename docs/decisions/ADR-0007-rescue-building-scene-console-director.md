# ADR-0007: Rescue-building scenario, Husky drive model, operator console and camera director

- **Status:** Accepted (verified in the `aegisnav-jazzy` container, headless and with the Gazebo GUI under Xvfb, 2026-10-29). Not yet run on the target laptop's GPU.
- **Context:** The corridor world (ADR-0005/0006) proved the data path but was judged "a simple test only": one target, a primitive robot, and evidence spread across terminal logs and RViz. A presentable mission needed a larger multi-goal scenario, a recognisable robot, and a single view showing *why* the robot acted. Doing that must not weaken AGENTS.md §4 rules 4, 6 and 9: occupancy, risk and visuals stay separate, and geometry must stay aligned.

## Decision

1. **The scenario is defined in the core, and the world is generated from it.** `aegisnav_core.scenarios.rescue_building_scenario()` is a 40×28 grid at 0.5 m. It defines the base, two victims, the approach waypoints, three refuges, fire and gas sources, and a demo script: at 8 s two ignitions on the first-leg route, and at 105 s a confidence drop to 0.42. `test_scenarios` checks that the ignitions sit on the planned route and that a detour of at least 5 cells exists, so the script cannot silently become a no-op. An earlier version was a no-op: its ignition at 22 s was off-route. `tools/world_rescue_building.py` writes the SDF and both maps (0.5 m planning, 0.05 m localization) from that grid. `aegisnav_bringup/test/test_assets.py` checks alignment for every preset in `config/worlds.yaml`. Doors are at least 3 cells wide (`min_passage_cells`, tested).
2. **Only CC0 assets, pinned and verified** (since 2026-10-30 also CC BY 4.0 with recorded attribution: the `Fog Emitter` texture, item 8). 14 CC0 Open Robotics Fuel models are vendored by `tools/fetch_fuel_assets.py`, which checks the license and SHA-256 (`THIRD_PARTY_NOTICES.md`). Every included model is `<static>`: with dynamic props the real-time factor was measured at 0.008.
3. **The Husky drive model is a true differential drive.** The Fuel Husky meshes are used for visuals only. Drive comes from two hidden centre wheels plus frictionless casters, raised 4 mm. A four-wheel skid-steer model could not rotate in place under DART contact. The visual wheels are driven so they appear to turn. Measured AMCL error after the change: 0.03 m.
4. **Clearance is a hard constraint at doors.** `PlannerConfig.inflate_obstacles_cells` (0–3) makes cells within N of a wall untraversable. The exception is the goal cell, where a soft cost remains (`wall_clearance_cells`/`wall_clearance_penalty_m`). With `straight_near_obstacles`, diagonal moves next to walls are forbidden. The rescue_building preset uses `inflate=1, clearance=2, penalty=1.0, straight=true`. `evaluate_route` applies the same rules, so a replayed route cannot cut a corner the planner would refuse. This was needed because the 0.6 m wide Husky clipped door jambs with soft costs alone.
5. **Operator console (`aegisnav_ros/console_node.py`, stdlib HTTP server only).**
   - **Input:** the console publishes the same simulated inputs as RViz/CLI (`/clicked_point`, `/goal_pose`, `/aegisnav/perception_confidence`). It never feeds the planner by any other path.
   - **Output:** the map, the current and +N s forecast, the route, a cancelled-route ghost, the decision and its reason code, the events and the mission metrics.
   - **Comparison:** the "shortest path (ignores hazard)" line is computed by the planner on the same forecast (`decision.comparison`), not by the browser.
   - **Network:** the console binds inside the container; `tools/run_gui.sh` publishes it to `127.0.0.1:8080` only. No external resources are loaded.
6. **Camera director (`director.py` logic, `director_node.py` wiring).** The camera shows an overview for the first 6 s, then a chase view. Each ignition at a new position (on `/aegisnav/hazard_ignition` or `/clicked_point`) triggers a 5 s cut-away to the fire. Each change of decision `action` triggers a 5 s top-down tactical view; it does not interrupt a fire cut-away. It drives only the Gazebo GUI camera (`/gui/track`, `/gui/move_to/pose`). It is never started headless.
7. **Recording layout** (`config/gazebo_gui_recording.config`). This is the installed gz-sim 8 default GUI, keeping only the 3D scene, camera tracking, markers, world control and stats. The entity tree, component inspector and editing tools are removed.
8. **Hazard source effects are particle emitters above the lidar, over flagged geometry** (revised 2026-10-30; display only).
   - **Earlier attempt (2026-10-29):** particle emitters with a generated puff texture and no colour-range image rendered as opaque squares on Mesa/ogre2 and were removed.
   - **Current effect:** the emitters use the CC BY 4.0 Fuel `Fog Emitter` v5 alpha texture `fog.png` and generated colour-over-lifetime ramps (`models/aegisnav_effects`, written by `tools/generate_world_assets.py`). Fire = flame tips + grey smoke column; gas = a hovering green cloud. In GUI frames under Xvfb with Mesa iris they render as soft, coloured puffs.
   - **Texture paths are absolute** (from the installed `aegisnav_bringup` share). The renderer reported `model://…` colour-range images as "not found" for a model spawned from an SDF string, and the particles were then grey.
   - **Lidar finding:** emitters cannot carry visibility flags, and `particle_scatter_ratio` 0 (also 0.0001) did **not** hide them from the `gpu_lidar`. With the lowest particle edge 0.04–0.09 m above the scan plane, the forward range fell from 3.33 m to 0.19–1.34 m. At 0.49 m above, it was unchanged. So every particle starts `PARTICLE_CLEARANCE_M` (0.45 m) above `LIDAR_PLANE_Z_M` (0.55 m, the Husky; the corridor robot is at 0.28 m; a test reads both heights from the world files), and a unit test enforces this. Everything lower (ember bed, translucent flame cones, gas floor haze and column) is geometry carrying visibility flag `0x2`. `particle_scatter_ratio` stays 0 but is not relied on.
   - **Verified:** `check_visibility_mask.py` passes in the corridor world (3.33 m with the fire effect, same as empty). In `rescue_building`, fire and gas placed 0.6 m and 0.3 m ahead of the Husky lidar left the forward range unchanged over 10 scans (4.55–4.56 m). The check inspects the forward ±3 beams only.
   - **Limits:** `gz-sim-particle-emitter-system` is in the generated `rescue_building` world only; the hand-written corridor world does not load it, so there it shows only the flagged geometry. Native NVIDIA rendering is not tested. This is a visual effect, not combustion or dispersion physics.

## Consequences

- The scene, maps and planner share one geometry source. Visual props cannot silently diverge from the planning grid; the asset test fails instead.
- The console is a viewer plus a simulated-input panel, not a control station. All its inputs are labelled simulated.
- The planner cycle on the 40×28×91 forecast takes about 1.1–1.3 s in the container while continuing. Under degraded confidence, where refuges are also assessed and the hazard has spread, it rose to 1.5–2.3 s. The planner logs a warning above 1 s. This bounds how fast a new hazard can change the route; caching or incremental search is the next optimisation if it matters.
- The source effects and tiles show the forecast; they are not fire or gas physics (ADR-0002). The `Fog Emitter` texture is CC BY 4.0 and must be credited (`THIRD_PARTY_NOTICES.md`, video credits).

## Amendment 2026-10-30: prop footprints must match the grid

In the first `rescue_compound` mission, the Husky drove into a pallet stack at cell (58, 39) and stayed pinned there. Its wheels kept turning, so wheel odometry, and AMCL with it, drifted 7 m from the true pose (measured against Gazebo ground truth). The cause was a wrong comment in the generator: it assumed the `euro_pallet` mesh origin was a corner. The mesh, with its node transform applied, is centred, so every pallet was shifted 0.6 m onto free cells.

Rescue Randy is also larger than its 0.5 m victim cell (about 0.75 × 1.0 m, off-centre), and its collision uses the same mesh.

Fixes:
- **Pallets:** centred, resting on the floor, in clusters at least as large as the pallet.
- **Victims:** turned so the body lies on blocked cells, with a corner of rubble for the courtyard victim.
- **Tests** (`test_assets`):
  - the generator's footprint constants must equal the bounds measured from the mesh files (node matrices, DAE unit and model scale applied);
  - no pallet may cover a free cell;
  - no victim may reach more than 0.1 m into free space. The planner's 1-cell inflation keeps the robot body at least about 0.24 m from blocked cells.
