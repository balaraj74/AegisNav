# Data contracts and coordinate conventions

## Grid geometry

- Coordinates use integer grid cells `Cell(x, y)`, with `(0,0)` at the bottom-left of the occupancy grid.
- Flattened index is `y * width + x`, matching `nav_msgs/OccupancyGrid.data` row-major layout.
- World coordinates for the sample map use an axis-aligned origin: `origin_x + (x + 0.5) * resolution`, `origin_y + (y + 0.5) * resolution`.
- The initial ROS converter rejects rotated or non-planar grid origins rather than silently applying incorrect coordinates. Implement and test a complete coordinate transform before removing this restriction.
- Static cells are occupied if value is negative (unknown) or at/above configurable `occupied_threshold`, which defaults to 50.

## Risk and uncertainty

| Field | Range | Meaning | Not equivalent to |
|---|---:|---|---|
| `risk[i]` | 0–1 | Normalized simulated hazard severity at cell `i` | Occupancy, physical concentration or calibrated probability |
| `uncertainty[i]` | 0–1 | How uncertain the simulated estimate is; higher is less certain | Perception model confidence or statistical confidence interval |
| `perception_confidence` | 0–1 | External confidence scalar supplied to the mission policy; higher is better | A calibrated confidence score unless a real estimator and calibration are implemented |

`HazardFrame` keeps risk and uncertainty arrays the same length as `width * height`. Invalid values or mismatched arrays must raise an error at the boundary.

## Forecast timing

`HazardForecast.time_step_s` is the simulated seconds between adjacent forecast layers. Layer 0 is the current risk estimate. Layer `i` describes the forecast at `i * time_step_s` seconds. The planner clamps to the last frame if its route extends beyond the forecast horizon; this is documented tail behavior, not a guarantee that the world remains static after the horizon.

For the sample configuration there are 121 frames: the current estimate plus 120 future steps at 1 second each. This horizon is chosen to cover the sample route’s outbound-and-return forecast time with per-move time discretization. The source cells are grid indices for the supplied 24×16 static map, not world coordinates and not values derived from sensors.

## Route output

A successful `RoutePlan` includes an ordered tuple of cells and route statistics. A failed plan returns `success=false` and a stable reason, for example `no_safe_path`, `start_or_goal_blocked`, `start_or_goal_outside_map` or `missing_forecast`. Do not treat an empty path as success.

## Mission actions

- `continue_to_target`: the predicted outbound and return plans satisfy the configured rules.
- `replan`: reserved for future explicit event-driven transition logic; the current supervisor returns the selected operational action directly rather than using it as a generic status.
- `slow_and_reassess`: perception confidence is degraded; the preferred route is a configured low-exposure observation/refuge point.
- `return_to_base`: the mission should be interrupted in favor of the base route.
- `seek_refuge`: select a reachable refuge when target/return feasibility is rejected or perception is critical.
- `abort`: no acceptable route is available for the chosen policy.

Actions are policy outputs; they do not indicate physical execution. Execution state must be tracked separately by any Nav2 action client.

## Execution state (2026-10-29)

`ExecutionController` (core, ROS-free) tracks execution separately from policy actions: `idle`, `dispatching`, `executing`, `cancelling`, `fault`. A replanned route replaces the active goal only after the old goal terminates; a route that rejoins the active one within `rejoin_cells` (2) cells is not a replan; `keep_active_route` keeps the active route unless the candidate is at least `replan_min_improvement` (0.15, fraction) cheaper under the same forecast or the active route has become unacceptable. Three consecutive goal rejections/failures latch `fault` (no further dispatch until reset). `replan` therefore appears as cancel + `replacement_dispatch` in the execution trace, not as a `MissionAction`.

## Static traversability vs hazard

`PlannerConfig.wall_clearance_penalty_m` (default 0.3, metres-equivalent per entered cell that touches an obstacle in its 8-neighbourhood) is a geometric preference derived from the static occupancy grid. It is added in the same edge-cost function as hazard and uncertainty but is reported separately from risk and never stored in hazard layers.

## ROS topics (as run in the Jazzy container, 2026-10-29)

| Topic / action | Type | Frame | Producer → consumer | Units / semantics | Failure behaviour |
|---|---|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` (transient local) | `map` | `map_server` → AMCL, local costmap, RViz | 0.05 m, generated from the core grid | — |
| `/aegisnav/planning_map` | `nav_msgs/OccupancyGrid` (transient local) | `map` | `planning_map_server` → planner, forecaster, console | planning grid, same geometry (corridor 0.4 m, rescue_building 0.5 m) | map rejected on bad metadata; nothing planned |
| `/aegisnav/hazard_forecast` | `aegisnav_msgs/HazardForecast` | `map` | forecaster → planner, console | `horizon_steps + 1` layers `time_step_s` apart (corridor 121 × 1 s, rescue_building 91 × 2 s), risk/uncertainty 0–100 ↔ [0,1] | stale (>3 s) → abort decision, goal cancelled |
| `/aegisnav/hazard_ignition` | `geometry_msgs/PointStamped`, 1 Hz once active | `map` | scenario → forecaster | x,y metres; intensity is forecaster param `ignition_intensity` [0,1] | wrong frame / wall / outside map → ignored, logged |
| `/aegisnav/perception_confidence` | `std_msgs/Float32`, 2 Hz | — | scenario → planner | simulated scalar [0,1], not a perception output | outside [0,1] ignored |
| `/goal_pose` | `geometry_msgs/PoseStamped`, 0.5 Hz | `map` | scenario → planner | target cell centre; base after `at_destination` at target | frame mismatch → planning cycle rejected |
| `map→base_footprint` TF | — | — | AMCL (`map→odom`) + gz DiffDrive (`odom→base_footprint`) → planner | pose; freshness = stamp of `odom→base_footprint` | stale/missing → abort decision, goal cancelled |
| `/aegisnav/planned_path` | `nav_msgs/Path` | `map` | planner → RViz | centred, densified (0.1 m) waypoints; empty path on abort | — |
| `/aegisnav/decision` | `std_msgs/String` (JSON) | — | planner → scenario, console, director, logs | `action`, `reason_code`, `explanation`, `at_destination`, `robot_cell`, `goal_cell`, route stats, `comparison` (AegisNav vs shortest path on the same forecast: exposure, peak risk, length, `crosses_hard_limit`, `route_xy`) | scenario ignores a decision whose `goal_cell` is not its current goal |
| `/aegisnav/execution_event` | `std_msgs/String` (JSON) | `map` | planner → visualization, console | `event` ∈ `send`/`cancel`, `reason_code`; `route_xy` (metres) on `send` | — |
| `/aegisnav/mission_status` | `std_msgs/String` (JSON), 1 Hz | — | scenario → console | `goals_total`, `goals_reached` (indices), `current_goal` (cell), `phase` ∈ `outbound`/`returning`/`user_goal`, `elapsed_s` (mission clock from first AMCL pose) | — |
| `/follow_path` | `nav2_msgs/action/FollowPath` | `map` | planner → `controller_server` (Regulated Pure Pursuit) | `controller_id=FollowPath`, `goal_checker_id=general_goal_checker`, `progress_checker_id=progress_checker` | non-success status → `failed`; 3 in a row → fault latch |
| `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/clock` | standard | — | ros_gz_bridge ↔ Gazebo | remapped from `/model/aegis_bot/*` | — |

### Onboard sensing (`sensing:=true`, ADR-0008)

| Topic | Type | Producer → consumer | Units / semantics | Failure behaviour |
|---|---|---|---|---|
| `/aegisnav/hazard_truth` | `aegisnav_msgs/HazardForecast` | forecaster `aegisnav_hazard_truth` (`source_mode: truth`) → sensor, visualization, console "Truth" layer | the hidden simulated field (configured sources + ignitions); **never** read by the planner | — |
| `/aegisnav/hazard_observation` | `nav_msgs/OccupancyGrid`, 2 Hz | `aegisnav_hazard_sensor` → forecaster (`source_mode: belief`) | 0..100 = observed risk ×100 in cells within 4 m and line of sight (walls and dense smoke, true risk ≥ 0.6, occlude); −1 = not seen this scan; seeded noise σ 0.03 | no fresh TF pose (> 1 s) or misaligned truth → nothing published; a misaligned observation is rejected by the forecaster |
| `/aegisnav/hazard_forecast` (sensing) | `aegisnav_msgs/HazardForecast` | forecaster in `belief` mode → planner, console | forecast from what has been seen; unseen cells: risk 0, uncertainty 0.6 | as before |
| `/aegisnav/hazard_discovered` | `geometry_msgs/PointStamped` | belief forecaster → console, director | centre of a burning cell (risk ≥ 0.5) with no cell ever seen burning within 2 cells (a new hazard region) | — |
| `/aegisnav/victim_detections` | `std_msgs/String` (JSON), 2 Hz | sensor → console | `{"found": [[x, y], ...]}` victim cells seen so far (grid line of sight, not vision) | — |
| `/aegisnav/perception_confidence` (sensing) | `std_msgs/Float32`, 2 Hz | sensor → planner (the scenario node stops publishing it) | `min(visible / clear-air visible, 1 − AMCL position σ / 0.5 m)`; a heuristic, not calibrated | no AMCL pose yet → nothing published |

The planner applies hysteresis to confidence: after it drops below `minimum_continue_confidence` (0.55), the mission continues only once it is back to `resume_confidence` (0.65). The decision JSON reports the measured value.

`/aegisnav/mission_status` also carries `unreachable` (goal cells set aside by triage). With `triage:=true` the scenario node chooses the next inspection point as the lowest-exposure safe route on the latest `/aegisnav/hazard_forecast`. When the planner reports `target_route_unavailable`, it switches once to another reachable point and retries the set-aside one later.

### Visualization topics/services (display only — never consumed by planning)

| Name | Type | Producer → consumer | Semantics |
|---|---|---|---|
| `/aegisnav/hazard_markers/current` | `visualization_msgs/MarkerArray` | forecaster → RViz | layer 0, cells with risk ≥ `marker_min_risk` (0.25) |
| `/aegisnav/hazard_markers/predicted` | `visualization_msgs/MarkerArray` | forecaster → RViz | layer `predicted_marker_step` (10 → +10 s), violet, raised 0.28 m |
| `/aegisnav/mission_markers` | `visualization_msgs/MarkerArray` | visualization → RViz | base, refuges, current goal, decision text |
| `/world/aegisnav_rescue_world/create`, `/remove` | `ros_gz_interfaces/srv/SpawnEntity`, `DeleteEntity` (bridged) | visualization → Gazebo | one hazard-tile model per band, refreshed every 2 s |

### Operator console (`aegisnav-console`, port 8080; `tools/run_gui.sh` publishes it on 127.0.0.1 only)

| Endpoint | Direction | Payload | Effect |
|---|---|---|---|
| `GET /api/static` | out | grid size, resolution, origin, blocked cells, base/refuges/victims/goals (cells) | — |
| `GET /api/stream` | out | server-sent events, 4 Hz: robot pose, route, ghost, current and +N s risk, decision, mission status, metrics, events | — |
| `POST /api/fire` | in | `{"x": m, "y": m}` (map frame) | publishes `/clicked_point` (simulated ignition) |
| `POST /api/goal` | in | `{"x": m, "y": m}` | publishes `/goal_pose` |
| `POST /api/confidence` | in | `{"value": 0..1}` | publishes `/aegisnav/perception_confidence` (simulated scalar) |

Invalid bodies (size > 4 KiB, non-finite numbers, confidence outside [0,1]) return HTTP 400 and publish nothing.

### Recording camera (`aegisnav-director`, GUI runs only)
Subscribes to `/aegisnav/decision`, `/aegisnav/hazard_ignition` and `/clicked_point`. Drives only the Gazebo GUI camera, via the gz transport topic `/gui/track` (`gz.msgs.CameraTrack`) and service `/gui/move_to/pose` (`gz.msgs.GUICamera`); each shot is sent three times over 1 s because a one-shot topic publish can be lost. It has no effect on the simulation.
