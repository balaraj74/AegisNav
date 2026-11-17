# 3D simulation assets: practical route to stronger visuals

## What the starter contains

`aegisnav_bringup/worlds/aegisnav_rescue.sdf` is a lightweight sample scene authored from primitive geometry. It contains a bounded rescue test area, partition walls, two hazard visual anchors, refuge markers and a simple differential-drive robot with a simulated lidar. It is useful as a known geometric test world, **not as a photorealistic scene**.

Its matching static occupancy map is `aegisnav_bringup/maps/aegisnav_rescue.yaml` + `.pgm`. Keep these in sync when geometry changes. Hazard source cell coordinates in `aegisnav_ros/config/aegisnav.yaml` assume this map; re-index them if you change maps.

## Recommended reuse option: `disaster_world`

The upstream [`juanscelyg/disaster_world`](https://github.com/juanscelyg/disaster_world) repository describes ROS 2 / Gazebo Harmonic disaster-response worlds based on Baylands Park and Nathan Benderson Park, with 44 reusable models and victim ground-truth YAML. Its README states ROS 2 Jazzy + Gazebo Harmonic (`gz-sim 8`) requirements and notes that first launch may download Fuel assets.

Use it as an optional visual environment, not a blind replacement:

1. Review its latest README, license, model dependencies and launch arguments.
2. Clone it into a separate workspace folder; do not copy its asset files into this project unless redistribution terms have been checked.
3. Build its `disaster_world` package on the target Jazzy/Harmonic machine.
4. Launch one world and confirm the actual map/world coordinates.
5. Export or create a matching occupancy map and place hazards/refuges using measured world-to-grid transforms.
6. Update source cells, base/refuge cells, world spawn pose and scenario tests together.
7. Rerun all core tests and make a saved visual regression artifact.

The upstream world is large and may require an initial internet connection for Fuel assets. Cache it before the competition and prepare the primitive fallback world so the demo is not dependent on network access.

## Isaac Sim consideration

Isaac Sim can produce high-fidelity rendering, but the target machine has an RTX 2050 with 4 GB VRAM and 16 GB system RAM. Treat it as an optional workstation/cloud environment, not a deadline-critical local dependency. Verify the current official minimum requirements before spending setup time.

## Render quality without destabilizing the simulator

- Prefer a stable 30–45 degree camera angle that makes the gate, hazard region and alternate route visible.
- Use clear key/fill lighting, high-contrast but restrained materials, and moderate shadows.
- Keep the simulation's collision geometry aligned with the occupancy map. Decorative rubble must not create hidden collisions.
- Let Gazebo show the world/robot and use RViz for risk layers and custom planned paths. Explicitly describe that they are two linked views, not one integrated photorealistic HUD.
- On a 4 GB GPU, reduce shadow quality, camera resolution, anti-aliasing and sensor update rates before attempting multiple high-cost renderers.
- Record an internal test in 720p first. Only increase quality after the exact launch survives a repeatable smoke test.

## Visual feature vs algorithm feature

A glowing orange patch in SDF is a visual anchor. The live forecast must come from `/aegisnav/hazard_forecast`; the planner must consume those values. A static flame that changes the camera but never affects the route is not an implemented hazard-aware feature.
