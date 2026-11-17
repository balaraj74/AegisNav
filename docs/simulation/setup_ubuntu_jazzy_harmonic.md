# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic setup

## Scope

This guide targets an Ubuntu 24.04 host (not a VM without GPU acceleration). Gazebo Harmonic is the ROS 2 Jazzy pairing. The sample world is intentionally modest for a laptop with an RTX 2050 4 GB and 16 GB RAM.

## 0. Preflight

```bash
lsb_release -ds
nvidia-smi
free -h
df -h ~
```

Use NVIDIA's proprietary driver if it is already installed and stable. Do not change drivers or Secure Boot settings immediately before the competition unless the current graphics stack is actually broken.

## 1. Source ROS 2 and install dependencies

If ROS 2 Jazzy is not installed, follow the current official [Ubuntu Debian installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Do not mix ROS distributions in one shell.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep
```

If package installation fails, inspect the exact apt output and compare it with the official [Gazebo Harmonic ROS integration guide](https://gazebosim.org/docs/harmonic/ros_installation/). Do not blindly add a second Gazebo repository; Jazzy's vendor packages are the preferred pairing.

## 2. Build the workspace

From the extracted project root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Sanity checks:

```bash
ros2 pkg list | grep -E '^(aegisnav_|ros_gz|nav2_)'
ros2 interface show aegisnav_msgs/msg/HazardForecast
gz sim --versions
```

`gz sim --versions` availability can vary with the installed CLI; if it is unsupported, use `gz sim --help` and `gz sim -v 3` to confirm the executable.

## 3. Launch sample world and forecast

```bash
source /opt/ros/jazzy/setup.bash
source ~/path/to/AegisNav-Starter/ros2_ws/install/setup.bash
ros2 launch aegisnav_bringup sim_world.launch.py
```

To open RViz as a separate visual review panel:

```bash
ros2 launch aegisnav_bringup sim_world.launch.py start_rviz:=true
```

Expected components:

- Gazebo Harmonic opens `aegisnav_rescue.sdf` and runs the sample differential-drive robot.
- `/clock` is bridged from Gazebo.
- `/model/aegis_bot/scan`, `/model/aegis_bot/odometry` and `/model/aegis_bot/cmd_vel` are bridged.
- `map_server` publishes the sample `/map` and lifecycle manager activates it.
- AegisNav publishes `/aegisnav/hazard_forecast` and `/aegisnav/hazard_markers`.
- With `start_rviz:=true`, the future risk layer and any planned path are viewable in RViz.

The sample launch does not start AMCL or the full Nav2 navigation stack. As a result, the risk planner waits for `/amcl_pose` and `/goal_pose` by design. The world alone should still be usable for checking SDF, clock, map alignment, and hazard forecasting.

## 4. Check map and forecast data

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/path/to/AegisNav-Starter/ros2_ws/install/setup.bash
ros2 topic echo --once /map
ros2 topic info /aegisnav/hazard_forecast
ros2 topic echo --once /aegisnav/hazard_forecast
ros2 topic echo --once /aegisnav/hazard_markers
```

Validate that `width`, `height`, `resolution`, origin and layer counts agree. `HazardForecast.risk_layers` and `uncertainty_layers` must have equal counts. Configured source cells `[11,3]` and `[9,4]` correspond to the sample map only.

## 5. Smoke-test a planning decision without AMCL

For a **planner-contract test only**, manually publish a sample pose and target in the `map` frame. Do not enable FollowPath execution for this test.

Pose corresponds to cell `(2,2)` whose center is `(-3.8, -2.2)`:

```bash
ros2 topic pub --once /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -3.8, y: -2.2, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}"
```

Target corresponds to cell `(21,13)` whose center is `(3.8, 2.2)`:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 3.8, y: 2.2, z: 0.0}, orientation: {w: 1.0}}}"
```

Then inspect:

```bash
ros2 topic echo --once /aegisnav/decision
ros2 topic echo --once /aegisnav/planned_path
```

This checks message-to-core conversion and path publication. It does not verify real localization or robot control.

## 6. Confidence perturbation

Send a synthetic perception-confidence signal and observe the decision mode:

```bash
ros2 topic pub --once /aegisnav/perception_confidence std_msgs/msg/Float32 "{data: 0.45}"
ros2 topic echo --once /aegisnav/decision
```

Try `0.2` for the critical-confidence branch. The core treats confidence as an externally provided test scalar; it does not estimate confidence from lidar/camera data.

## 7. Enabling optional FollowPath action execution

Do this only when all of these are true:

- the `FollowPath` action server is running and `ros2 action info /follow_path` confirms it;
- the robot localization pose and map share the `map` frame;
- static map, robot footprint, sensor topics, local costmap and controller parameters have been checked;
- the controller plugin IDs in `aegisnav.yaml` match the actual Nav2 configuration;
- a human has verified a slow-speed test in an empty world.

Change `execute_with_follow_path: false` to `true` in the ROS parameter file or pass an override after confirming the exact launch syntax for the installed ROS 2 version. Start with low velocity limits and a reachable goal. Immediately test replanning and abort cancellation. If any of the above is unknown, leave it disabled and record paths/decisions only.

## 8. Performance advice for the 4 GB GPU

- Keep Gazebo at 720p, moderate shadows and a single camera.
- Launch RViz only when needed: `start_rviz:=false` by default.
- Avoid launching multiple simulators or high-resolution GPU lidar/camera suites simultaneously.
- Cache any Fuel models before the competition day.
- Keep the local deterministic demo available as a fallback in case graphics or ROS integration fails.

## 9. Troubleshooting

| Symptom | Check |
|---|---|
| No `/map` | map server lifecycle state, installed map YAML/PGM path and `use_sim_time` |
| Hazard node reports source outside/inside occupied cell | map dimensions and `source_cells_xy`; source coordinates are grid indices, not meters |
| Forecast published but no path | planner needs `/amcl_pose` and `/goal_pose`; check `ros2 topic info` and message frame IDs |
| Planner rejects map orientation | current adapter deliberately supports axis-aligned map origins only |
| RViz fixed-frame warning | set fixed frame to `map`, confirm map header frame and timestamps |
| Robot does not move | path execution is disabled by default; inspect FollowPath server and controller stack before enabling |
| GPU stutter | close RViz, reduce resolution/shadows, lower sensor rate, run the core demo separately |
| Asset world has missing models | launch/cache the upstream Fuel models while internet is available; keep sample fallback world |
