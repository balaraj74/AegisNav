#!/usr/bin/env bash
# Inside the Jazzy container: sandbox mode (no scripted events). Performs the same inputs a user
# makes in RViz — goal, fire on the active route, confidence drop — and prints what the system did.
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
RVIZ=false
if [ -n "${RVIZ_SHOT:-}" ]; then  # optional: RViz on a virtual display, screenshot after the fire
  Xvfb :99 -screen 0 1400x900x24 > /tmp/xvfb.log 2>&1 & export DISPLAY=:99; sleep 2; RVIZ=true
fi
ros2 launch aegisnav_bringup sim_world.launch.py headless:=true scenario:=false start_rviz:=$RVIZ > /tmp/sim.log 2>&1 &
ros2 topic echo /aegisnav/execution_event std_msgs/msg/String --field data > /tmp/events.log 2>&1 &
sleep 22
echo "== 1. goal: target cell (21,13)"; ros2 topic pub -r 2 -t 4 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 3.8, y: 2.2}, orientation: {w: 1.0}}}' >/dev/null
sleep 10
ROUTE=$(python3 - <<'PY'
import json
evs=[json.loads(b.strip().strip("'")) for b in open('/tmp/events.log').read().split('---') if b.strip().startswith(("'{", '{'))]
sends=[e for e in evs if e['event']=='send']
print(json.dumps(sends[-1]['route_xy']) if sends else '[]')
PY
)
echo "   active route: $ROUTE" | cut -c1-200
# Fire on a cell of the active route ahead of the robot (the user would click there).
read FX FY <<< $(python3 -c "import json; r=json.loads('$ROUTE'); p=r[max(0, len(r)-5)]; print(p[0], p[1])")
echo "== 2. fire on the active route at ($FX, $FY)"; ros2 topic pub -r 2 -t 3 /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: $FX, y: $FY}}" >/dev/null
export FIRE="$FX $FY"
sleep 12
[ -n "${RVIZ_SHOT:-}" ] && import -window root /out/rviz_sandbox_after_fire.png
echo "== 3. confidence 0.2"; ros2 topic pub -r 2 -t 6 /aegisnav/perception_confidence std_msgs/msg/Float32 '{data: 0.2}' >/dev/null
sleep 20
echo "== system reaction (decisions, execution events, fires)"
sed -E 's/\x1b\[[0-9;]*m//g' /tmp/sim.log | grep -E "Decision |ignition accepted|Cancelling|Reached the goal" | sed -E 's/^\[[a-z_0-9-]+\] \[[A-Z]+\] \[([0-9.]+)\] \[[a-z_]+\]: /\1 /' | cut -c1-150
python3 - <<'PY'
import json
evs=[json.loads(b.strip().strip("'")) for b in open('/tmp/events.log').read().split('---') if b.strip().startswith(("'{", '{'))]
import os
fire = [float(v) for v in os.environ.get("FIRE", "nan nan").split()]
for e in evs:
    extra = ""
    if e['event'] == 'send':
        r = e['route_xy']
        through_fire = any(abs(x - fire[0]) < 0.01 and abs(y - fire[1]) < 0.01 for x, y in r)
        extra = f"route {len(r)} cells, end {r[-1]}, passes fire cell: {through_fire}"
    print("event", e['event'], e['reason_code'], extra)
PY
