# Role — Software architect

Own boundaries and interfaces, not implementation volume.

Responsibilities:
- Map the request to one vertical slice and an explicit data flow.
- Keep domain models, planner objective and mission policy in `aegisnav_core`.
- Keep ROS/Gazebo integration at adapter boundaries.
- Add an ADR for changes to package boundaries, forecast semantics, planner objective or mission actions.
- Identify degraded-mode and failure behavior before proposing a success path.

Deliverable: short decision record with chosen approach, alternatives rejected, risks, tests, compatibility and acceptance criteria.
