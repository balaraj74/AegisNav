#!/usr/bin/env bash
# Send a test input to the running GUI simulation (tools/run_gui.sh). All inputs are simulated.
#   tools/sim_cmd.sh confidence 0.4     # perception confidence in [0,1]
#   tools/sim_cmd.sh fire X Y           # start a fire at map coordinates (metres)
#   tools/sim_cmd.sh goal X Y           # set the mission goal (metres)
#   tools/sim_cmd.sh status             # print the latest decision JSON
set -euo pipefail
NAME=aegisnav_gui
if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "No running '$NAME' container. Start it with tools/run_gui.sh first." >&2
  exit 2
fi
num='^-?[0-9]+([.][0-9]+)?$'
ros() { docker exec "$NAME" bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && $*"; }
case "${1:-}" in
  confidence)
    [[ "${2:-}" =~ $num ]] || { echo "usage: $0 confidence VALUE" >&2; exit 2; }
    ros "ros2 topic pub -r 2 -t 6 /aegisnav/perception_confidence std_msgs/msg/Float32 '{data: $2}' >/dev/null"
    echo "confidence set to $2 (published for 3 s; the planner keeps the last value)";;
  fire)
    [[ "${2:-}" =~ $num && "${3:-}" =~ $num ]] || { echo "usage: $0 fire X Y" >&2; exit 2; }
    # A few repeats: a single --once message can be lost before discovery; repeats are de-duplicated.
    ros "ros2 topic pub -r 2 -t 3 /clicked_point geometry_msgs/msg/PointStamped '{header: {frame_id: map}, point: {x: $2, y: $3}}' >/dev/null"
    echo "fire requested at ($2, $3)";;
  goal)
    [[ "${2:-}" =~ $num && "${3:-}" =~ $num ]] || { echo "usage: $0 goal X Y" >&2; exit 2; }
    ros "ros2 topic pub -r 2 -t 4 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: $2, y: $3}, orientation: {w: 1.0}}}' >/dev/null"
    echo "goal set to ($2, $3)";;
  status)
    ros "timeout 5 ros2 topic echo /aegisnav/decision --once --field data";;
  *)
    sed -n '2,7p' "$0"; exit 2;;
esac
