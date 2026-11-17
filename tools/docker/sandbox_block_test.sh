#!/usr/bin/env bash
# Inside the Jazzy container: set the target, then set fire to both openings of the divider wall.
# Expected (policy): no acceptable route to the target -> robot is redirected to a reachable refuge.
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
ros2 launch aegisnav_bringup sim_world.launch.py headless:=true scenario:=false > /tmp/sim.log 2>&1 &
sleep 22
ros2 topic pub -r 2 -t 4 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 3.8, y: 2.2}, orientation: {w: 1.0}}}' >/dev/null
sleep 4
for P in "-0.2 -1.4" "-0.2 1.4" "-0.2 1.8"; do  # lower gate's open cell (11,4) + both upper gate cells
  set -- $P; ros2 topic pub -r 2 -t 3 /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: $1, y: $2}}" >/dev/null
done
sleep 30
sed -E 's/\x1b\[[0-9;]*m//g' /tmp/sim.log | grep -E "Decision |ignition accepted|Cancelling|Reached the goal" | sed -E 's/^\[[a-z_0-9-]+\] \[[A-Z]+\] \[([0-9.]+)\] \[[a-z_]+\]: /\1 /' | cut -c1-150
