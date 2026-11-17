# ADR-0003: Publish a candidate route before enabling robot motion

- **Status:** Accepted
- **Context:** Incorrect frames, map alignment or action-controller IDs can produce unsafe motion even if A* is correct.
- **Decision:** `execute_with_follow_path` defaults to `false`. The ROS planner publishes `/aegisnav/planned_path` and a decision trace by default. Action execution is opt-in and must be tested against the actual Nav2 controller server.
- **Consequences:** The algorithm can be reviewed in RViz without moving the simulated robot. Additional setup is required for a full navigation demonstration. Abort/no-route decisions must clear the stale path and cancel an active FollowPath action when execution is enabled.
