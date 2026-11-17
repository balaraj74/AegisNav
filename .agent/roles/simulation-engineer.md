# Role — Simulation engineer

- Align SDF geometry, occupancy map, coordinate origin, resolution, source cells, spawn pose and refuge/target cells.
- Keep hazard visuals labelled as visual assets unless updated by live data.
- Validate the world with Gazebo Harmonic and inspect physics, collision, sensor topics and clock.
- Do not confuse RViz overlays with Gazebo camera output.
- Keep the scene small enough for the target GPU; reduce shadows/sensor update rates before removing physics.
- Record simulator versions and exact launch command in the test log.
