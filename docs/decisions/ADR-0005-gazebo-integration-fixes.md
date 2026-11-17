# ADR-0005: Gazebo/Nav2 integration choices made from observed failures

- **Status:** Accepted (each item verified in the `aegisnav-jazzy` container, headless Gazebo Harmonic, 2026-10-29)
- **Context:** The first live runs failed in ways unit tests could not show. Each decision below records the observation that forced it.
- **Decisions:**
  1. **Two-cell (0.8 m) openings** (`corridor_hazard_spread_v2`). One-cell (0.4 m) gaps left ~8 cm per side for the 0.23 m robot; the controller timed out ("patience exceeded"). PGM and SDF walls are now generated from the core grid by `tools/generate_world_assets.py`.
  2. **Robot model geometry.** Wheel joint axes were expressed in the rolled wheel frame (spin axis vertical): commanded forward motion drove sideways and pitched 43°. Axes are now `expressed_in="__model__"`, joints placed at wheel centres. A 15 mm-short rear caster pitched the robot 7.6° at rest, and with mass over the axle braking tipped it ~27° so the lidar hit the floor (phantom obstacles 0.3–0.45 m ahead). Casters are now level with the wheels, frictionless, front and rear. Measured pitch after the fix: 0.00° under step acceleration/braking.
  3. **Separate localization map.** AMCL on the 0.4 m planning grid mislocalized by 0.31 m (vs Gazebo ground truth). `/map` is now the same geometry at 0.05 m; the planner/forecaster read `/aegisnav/planning_map`. Measured AMCL error afterwards: 0.07–0.11 m.
  4. **Regulated Pure Pursuit** instead of DWB for `FollowPath`, with cost-regulated speed near obstacles. DWB with untuned critics drove into walls; RPP tracks an external path and keeps its own costmap collision check (local collision checking is not bypassed). Parameter names were verified from the installed Jazzy library.
  5. **Path presentation at the ROS boundary:** waypoints beside a wall are shifted half a cell toward free space (stays inside free cells) and densified to 0.1 m; the planner's start pose snaps to a free cell within 0.25 m (AMCL noise beside walls) and fails closed beyond that.
  6. **Planner hysteresis and wall-clearance preference** (see data contracts) to stop replan churn and keep the body off walls.
  7. **Pose freshness from `odom→base_footprint`**, because AMCL's `map→odom` is not refreshed while stationary.
  8. **Isolated test containers** (`--network none`, in-container timeout): leftover containers from timed-out runs were joining the ROS graph and corrupting later runs.
- **Consequences:** Closed-loop numbers from before these changes (map v1) are superseded. The kinematic runner and Gazebo now share map, scenario and policy; they do not share dynamics.
