#!/usr/bin/env bash
# Stage 3 smoke test (inside the Jazzy container): full headless launch, then observe.
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
ros2 launch aegisnav_bringup sim_world.launch.py headless:=true seed:=${SEED:-7} > /tmp/sim.log 2>&1 &
if [ -d /out ]; then
  ros2 bag record -s sqlite3 -o /out/bag /aegisnav/planned_path /aegisnav/decision /odom /cmd_vel /local_costmap/costmap /amcl_pose > /tmp/bag.log 2>&1 &
  BAG_PID=$!
fi
sleep 25
echo "=== nodes"; ros2 node list
echo "=== timing: gz real-time factor, sim clock vs latest odom stamp"
gz topic -e -t /stats -n 1 2>/dev/null | grep -m1 real_time_factor
python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
rclpy.init(); n = Node("timing_probe"); st = {}
n.create_subscription(Clock, "/clock", lambda m: st.__setitem__("clock", m.clock.sec + m.clock.nanosec * 1e-9), 10)
n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("odom", m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 10)
t0 = time.time()
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.1)
print("clock", st.get("clock"), "odom stamp", st.get("odom"), "lag", (st.get("clock") or 0) - (st.get("odom") or 0))
PY
echo "=== clock/scan/odom"
for t in /clock /scan /odom; do echo "$t: $(timeout 6 ros2 topic echo $t --once --no-arr 2>&1 | grep -m1 -E 'sec:|frame_id|WARNING' )"; done
echo "scan frame: $(timeout 6 ros2 topic echo /scan --once --field header.frame_id 2>&1 | head -1)"
echo "=== tf map->base_footprint"; timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m3 -E "Translation|Exception|Could not|frame"
echo "=== lifecycle"; for n in map_server amcl controller_server; do echo "$n: $(ros2 lifecycle get /$n 2>&1 | head -1)"; done
echo "=== actions"; ros2 action list -t
echo "=== probe (scan + local costmap)"; timeout 20 python3 /repo/tools/docker/probe_costmap.py 2>&1 | tail -3
ros2 topic echo /aegisnav/decision --field data > /tmp/dec.log 2>&1 &
echo "=== odom and map pose over time"
for i in $(seq 1 ${SAMPLES:-8}); do
  echo "t+$((i*5))s odom: $(timeout 4 ros2 topic echo /odom --once --field pose.pose.position 2>&1 | tr '\n' ' ' | cut -c1-70)"
  echo "        map: $(timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m1 Translation)"
  sleep 3
done
echo "=== ground truth vs amcl vs odom"
gz topic -e -t /world/aegisnav_rescue_world/pose/info -n 1 2>/dev/null | python3 -c "
import sys,re
for blk in sys.stdin.read().split('pose {')[1:]:
    if 'name: \"aegis_bot\"' in blk.split('position')[0]:
        P={k:float(v) for k,v in re.findall(r'([xyz]): (-?[0-9.e+-]+)',re.search(r'position \{(.*?)\}',blk,re.S).group(1))}
        print('gazebo truth (map frame = world frame): x=%.3f y=%.3f' % (P.get('x',0), P.get('y',0))); break"
echo "amcl map->base: $(timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m1 Translation)"
echo "odom (+ spawn -3.8,-2.2): $(timeout 4 ros2 topic echo /odom --once --field pose.pose.position 2>&1 | tr '\n' ' ' | cut -c1-70)"
echo "=== end probe"; timeout 25 python3 /repo/tools/docker/probe_costmap.py 2>&1 | grep -E "robot odom|planned path|lidar in map"
[ -d /out ] && cp /tmp/sim.log /tmp/dec.log /out/ 2>/dev/null
[ -n "${BAG_PID:-}" ] && kill -INT $BAG_PID && wait $BAG_PID; sleep 2
[ -d /out/bag ] && echo "=== bag analysis" && python3 /repo/tools/docker/analyze_bag.py /out/bag /tmp/sim.log
echo "=== decisions"; grep -oE '"action": "[a-z_]+"|"reason_code": "[a-z_]+"' /tmp/sim.log | head -0
echo "=== sim log: aegisnav + errors"
grep -E "aegisnav|ERROR|error|FollowPath|Cancel|fault|rejected" /tmp/sim.log | grep -v "process started" | cut -c1-240 | tail -40
