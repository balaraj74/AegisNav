# ROS smoke test on Ubuntu 24.04 (Jazzy)

Status: stages 0, 1 and 3 **executed** on 2026-10-29 in the `aegisnav-jazzy` Docker image (headless Gazebo); results in `docs/testing/verification_matrix.md`. On a native 24.04 install use the commands below; in Docker use `tools/docker/run_in_jazzy.sh` with `tools/docker/stage1.sh` / `stage3.sh`. Install ROS 2 Jazzy, `ros-jazzy-ros-gz`, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` per `setup_ubuntu_jazzy_harmonic.md`.

## Stage 0 — build and unit tests
```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
```

## Stage 1 — AegisNav nodes with no Gazebo and no Nav2 (adapter layer only)
Terminal A (map + lifecycle) and B (nodes; `execute_with_follow_path` stays false):
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$(ros2 pkg prefix aegisnav_bringup)/share/aegisnav_bringup/maps/aegisnav_rescue.yaml -p topic_name:=/aegisnav/planning_map &
ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
ros2 launch aegisnav_ros aegisnav_core.launch.py scenario:=true seed:=7 pose_source:=topic use_sim_time:=false
```
Terminal C: stand in for localization (frame must equal `map`), then watch:
```bash
ros2 topic pub -r 2 /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -3.6, y: -2.4}, orientation: {w: 1.0}}}}"
ros2 topic echo /aegisnav/decision          # expect continue_to_target, then seek_refuge ~16 s after first pose
ros2 topic echo /aegisnav/planned_path --once
```
Pass: decision JSON carries `reason_code`; after ~16 s the ignition log line appears; at ~28 s the decision becomes `slow_and_reassess`. (Pose never moves in this stage, so later decisions reflect only hazard and confidence.)

## Stage 2 — Nav2 controller + FollowPath (bring up your own localization/controller first)
Confirm, do not assume:
```bash
ros2 action list -t | grep -i follow          # exact action name and type
ros2 param get /controller_server controller_plugins
ros2 param get /controller_server goal_checker_plugins
ros2 param get /controller_server progress_checker_plugins
ros2 run tf2_ros tf2_echo map base_link
```
Then set `execute_with_follow_path:=true` and the confirmed names in `aegisnav.yaml`. Verify: goal accepted; ignition triggers cancel then (if route exists) a new goal; `ros2 topic echo /cmd_vel` goes to zero after cancel; abort sends no new goal; killing the controller server yields a logged fault, not a retry storm.

## Stage 3 — full launch (one command)
```bash
ros2 launch aegisnav_bringup sim_world.launch.py seed:=7 start_rviz:=true
```
Expected, in order: `/clock`, `/scan`, `/odom` flowing; `ros2 run tf2_ros tf2_echo map base_footprint` resolves; lifecycle managers activate `map_server`, `amcl`, `controller_server`; first decision on `/aegisnav/decision`; robot moves; ignition at the seeded time changes the decision. Items most likely to need correction: lidar `header.frame_id` (`lidar_frame` argument), bridge remap syntax, `tf2_ros_py` rosdep key, DWB parameter names, gz `Pose_V` TF content, whether `odom->base_footprint` is what gz publishes.
Stop: Ctrl-C the launch, then `pkill -f 'gz sim'` if any gz process remains (`pgrep -fa gz`).

Note (historical, superseded by Stage 3): `sim_world.launch.py` did **not** start AMCL/controller_server/costmaps; `config/nav2_params.yaml` exists but no launch uses it. That integration is still to be written and run on the target machine.

## Docker (what was actually run)
```bash
docker build -f tools/docker/Dockerfile.jazzy -t aegisnav-jazzy tools/docker      # ~5.6 GB image
tools/docker/run_in_jazzy.sh 'rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install && source install/setup.bash && colcon test && colcon test-result --verbose'
tools/docker/run_in_jazzy.sh 'colcon build --symlink-install >/dev/null && bash /repo/tools/docker/stage1.sh 45'
mkdir -m 777 /tmp/aegis_out && AEGIS_TIMEOUT_S=760 AEGIS_DOCKER_ARGS="-v /tmp/aegis_out:/out -e SEED=7" \
  tools/docker/run_in_jazzy.sh 'colcon build --symlink-install >/dev/null && SAMPLES=20 bash /repo/tools/docker/stage3.sh'
```
`/tmp/aegis_out` then holds `sim.log`, `dec.log` and a rosbag (`bag/`); `tools/docker/analyze_bag.py` reports path, pose and lethal costmap cells at the first controller collision warning.
