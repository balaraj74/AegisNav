#!/usr/bin/env bash
# Stage 1 smoke test (inside the Jazzy container, workspace built in /ws): adapter layer only.
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
MAP=$(ros2 pkg prefix aegisnav_bringup)/share/aegisnav_bringup/maps/aegisnav_rescue.yaml
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP -p topic_name:=/aegisnav/planning_map > /tmp/map.log 2>&1 &
sleep 4
ros2 lifecycle set /map_server configure; ros2 lifecycle set /map_server activate
ros2 launch aegisnav_ros aegisnav_core.launch.py scenario:=true seed:=7 pose_source:=topic use_sim_time:=false > /tmp/core.log 2>&1 &
sleep 4
ros2 topic pub -r 2 /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -3.6, y: -2.4}, orientation: {w: 1.0}}}}" > /dev/null 2>&1 &
ros2 topic echo /aegisnav/decision --field data > /tmp/dec.log 2>&1 &
sleep "${1:-45}"
echo "=== nodes"; ros2 node list
echo "=== core log"; grep -vE "^\s*$" /tmp/core.log | cut -c1-220 | head -40
echo "=== decisions (action/reason_code/t)"
python3 - <<'PY'
import json,re
prev=None
for block in open('/tmp/dec.log').read().split('---'):
    block=block.strip().strip("'\"")
    if not block: continue
    try: d=json.loads(block)
    except Exception as e: print('unparsed:',block[:120]); continue
    key=(d.get('action'),d.get('reason_code'))
    if key!=prev: print(key, 'conf',d.get('perception_confidence'), 'route', d.get('selected_route_success')); prev=key
PY
echo "=== path poses"; timeout 5 ros2 topic echo /aegisnav/planned_path --once 2>&1 | grep -c "position:"
echo "=== raw dec.log (first 600 chars)"; head -c 600 /tmp/dec.log
echo "=== one live decision"; timeout 8 ros2 topic echo /aegisnav/decision --once --field data 2>&1 | head -c 700
