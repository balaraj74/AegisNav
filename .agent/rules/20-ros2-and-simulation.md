# Rule 20 — ROS 2 and simulation contracts

- Baseline: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
- Keep ROS messages out of the core package.
- Use `map` frame for global paths; preserve frame IDs and timestamps.
- Treat map rotation (roll, pitch or yaw) as unsupported until a full coordinate transform is implemented and tested; do not ignore it silently.
- Occupancy values `<0` are unknown and blocked by default; values at/above the configured occupied threshold are blocked.
- Hazard grid data are normalized risk encoded as integer percentage `[0,100]`; unknown risk uses `-1` at interface boundaries.
- Forecast risk and uncertainty arrays must have the same shape as `/map` and the same layer count.
- Simulator hazard sources are configured scenario inputs, not live sensor observations.
- Execute navigation only when the configured Nav2 action server and controller plugins are verified; abort/no-route decisions must cancel active goals.
- Keep Gazebo hazard visuals and ROS forecast semantics distinct and document whether a visual is decorative or data-driven.
