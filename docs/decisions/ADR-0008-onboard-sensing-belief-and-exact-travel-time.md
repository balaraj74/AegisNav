# ADR-0008: Onboard sensing (the planner consumes a belief) and exact travel-time accounting

- **Status:** Accepted, 2026-10-30.
  - Sensing verified in a full headless Gazebo/Nav2 mission in the `aegisnav-jazzy` container, plus closed-loop core tests.
  - Exact time verified by the core test suite and the compound leg checks.
  - Neither has run on the target laptop yet.
- **Context:**
  - Until now the robot was omniscient. `hazard_forecast_node` knew every configured source and ignition, and the planner forecast from that perfect field. The user asked for a robot that "gets the information in real time and tackles it".
  - Separately, the planner rounded **each move** up to a whole forecast layer. With 0.5 m cells, 0.6 m/s and 2 s layers, a 0.83 s move counted as 2 s, overestimating travel time about 2.4×. On `rescue_building` this stayed inside the 90-layer horizon. On the larger `rescue_compound` it made a 35 s leg look like 84 s, and the warehouse leg was rejected as `forecast_horizon_insufficient`. It also meant risk was sampled later than the robot would actually be there.

## Decision

1. **Truth and belief are separate data products** (AGENTS.md §4.4).
   - With `sensing:=true`, a forecast node in `truth` mode publishes the hidden field on `/aegisnav/hazard_truth`. It drives only the Gazebo display and the simulated sensor.
   - `hazard_sensor_node` publishes `/aegisnav/hazard_observation` (`nav_msgs/OccupancyGrid`, −1 = not seen). It reports layer 0 of the truth only within `range_m` (4 m) and line of sight on the planning grid (`aegisnav_core.sensing.visible_cells`; walls occlude), with seeded Gaussian noise (σ 0.03).
   - The forecast node in `belief` mode (`HazardBelief`) forecasts only from what has been seen. Unseen cells get risk 0 but uncertainty 0.6, so unknown is not treated as safe: the planner's uncertainty weight penalises it.
   - **The planner and its topic contract are unchanged.**
2. **A discovery is a new hazard region:** a burning cell (risk ≥ 0.5) with no cell ever seen burning within 2 cells. Spread and sensor noise around the threshold are not re-announced. Each discovery is logged and published on `/aegisnav/hazard_discovered`, which the console and director show. Victims count as found when their cell is visible (`/aegisnav/victim_detections`).
3. **Exact travel time.**
   - Elapsed time accumulates exactly along each search path.
   - A move samples every layer from the first after its start time to the first at or after its arrival. It is still never earlier than the robot can be there, but no longer inflated per move.
   - The search state is (cell, arrival layer). States that share a layer keep the time of their cheapest path; if that makes a rescored route cross the threshold, the plan fails closed.
   - `_edge_cost` stays the single definition of the objective for both search and `evaluate_route`. `duration_steps` = ⌈travel time / layer⌉.

## Consequences

- **What the robot experiences:**
  - The robot can be surprised: in the live run it planned a route near the corridor fire, saw the fire at 19 s and rerouted at 22 s.
  - Its exposure can be higher than the omniscient planner's. That is the honest cost of not knowing.
  - The kinematic study adds mode `aegisnav_onboard_sensing` for a like-for-like comparison.
- **What the models are:**
  - The sensor, its range and noise, and the occlusion on a 0.5 m grid are simulation assumptions. They are not a thermal camera, a gas sensor or a calibrated detector.
  - Victim detection is grid line of sight, not vision.
- **Display:** Gazebo shows the truth, which viewers see. The console's "Hazard now" and "Predicted" layers show the belief, with a "Truth" layer for comparison. RViz markers show the belief.
- **Timing:** routes now report realistic durations, so more missions fit inside a given forecast horizon. The return-feasibility check is correspondingly less conservative, but it still uses the same hard threshold and exposure budgets. The existing tests, including the scripted-fire regression test and the closed-loop seeds, pass unchanged.
- **Equivalence:** the forecaster and A* speed-ups made the same day (neighbour tables, single-layer fast path) were proven bit-identical against the previous code before the time-accounting change. The time-accounting change itself intentionally changes results.

## Amendment 2026-10-30: computed confidence and its localization scale

With `sensing:=true`, perception confidence is `min(view quality, localization quality)`:
- **View quality:** cells visible now ÷ cells visible in clear air. Dense smoke (true risk ≥ 0.6) blocks sight.
- **Localization quality:** 1 − AMCL position σ ÷ 1.0 m.

The 1.0 m scale is tied to the 0.5 m planning cell. About one cell of position doubt (σ ≈ 0.45 m) reads as "degraded"; σ ≥ 0.7 m reads as "critical".

A first scale of 0.5 m was an unmeasured choice. Normal tracking (σ 0.10–0.18 m, error 0.05–0.3 m against Gazebo truth) then read only 0.64–0.80, too close to the 0.55 threshold.

The thresholds themselves are unchanged. The confidence latch (`resume_confidence` 0.65) prevents flapping. This is a heuristic, not a calibrated measure.
