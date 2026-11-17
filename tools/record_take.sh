#!/usr/bin/env bash
# Record one full rescue_compound mission with the Gazebo GUI on virtual displays.
# Usage: record2.sh <take> <mode: sensing|baseline> <seed> <xdisplay_gazebo> <xdisplay_console> <host_port>
# Output: ~/aegisnav_work/video/<take>/{gazebo.mkv,console.mkv,sim.log,metrics.json,capture_start.txt}
set -u
TAKE=$1; MODE=$2; SEED=$3; DG=$4; DC=$5; PORT=$6
OUT=${TAKES_DIR:-$HOME/aegisnav_work/video}/$TAKE; mkdir -p "$OUT"; ROOT=$HOME/AegisNav; NAME=aegisnav_rec_$TAKE
case "$MODE" in sensing) ARGS="sensing:=true" ;; baseline) ARGS="sensing:=false baseline:=true" ;; *) echo bad mode; exit 2 ;; esac
cleanup() { for p in ${F1:-} ${F2:-}; do kill -INT $p 2>/dev/null; done; sleep 3; docker rm -f $NAME >/dev/null 2>&1; kill ${CH:-} ${X1:-} ${X2:-} 2>/dev/null; }
trap cleanup EXIT
Xvfb :$DG -screen 0 1600x900x24 -ac +extension GLX >/dev/null 2>&1 & X1=$!
Xvfb :$DC -screen 0 1280x900x24 -ac >/dev/null 2>&1 & X2=$!
sleep 2
docker run -d --init --name $NAME -p 127.0.0.1:$PORT:8080 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e DISPLAY=:$DG -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$ROOT:/repo:ro" -v "$OUT:/out" --device /dev/dri \
  aegisnav-jazzy bash -lc "set -e; source /opt/ros/jazzy/setup.bash; mkdir -p /ws && cp -r /repo/ros2_ws/src /ws/src && cd /ws
    colcon build --symlink-install >/dev/null; source install/setup.bash
    ros2 launch aegisnav_bringup sim_world.launch.py world:=rescue_compound seed:=$SEED ignition_seed:=$SEED scenario:=true scripted_events:=true \
      start_rviz:=false record_path:=/out/metrics.json $ARGS" >/dev/null
for i in $(seq 1 180); do curl -s -o /dev/null http://127.0.0.1:$PORT/ && break; sleep 1; done
rm -rf "$HOME/aegisnav_work/chrome_$TAKE"
env -u WAYLAND_DISPLAY DISPLAY=:$DC XDG_SESSION_TYPE=x11 google-chrome --ozone-platform=x11 --user-data-dir="$HOME/aegisnav_work/chrome_$TAKE" \
  --no-first-run --no-default-browser-check --disable-gpu --window-position=0,0 --window-size=1280,900 --app=http://127.0.0.1:$PORT/ >/dev/null 2>&1 & CH=$!
sleep 6
ffmpeg -y -loglevel error -f x11grab -framerate 20 -video_size 1600x900 -i :$DG -c:v libx264 -preset ultrafast -crf 18 "$OUT/gazebo.mkv" & F1=$!
ffmpeg -y -loglevel error -f x11grab -framerate 10 -video_size 1280x900 -i :$DC -c:v libx264 -preset ultrafast -crf 22 "$OUT/console.mkv" & F2=$!
date +%s.%N > "$OUT/capture_start.txt"
END=$(( $(date +%s) + ${MAX_S:-1800} ))
while [ $(date +%s) -lt $END ]; do
  docker logs $NAME 2>&1 | sed -E 's/\x1b\[[0-9;]*m//g' > "$OUT/sim.log"
  if grep -q '"done": true' "$OUT/metrics.json" 2>/dev/null; then echo "recorder done"; sleep 10; break; fi
  docker ps -q -f name=$NAME | grep -q . || { echo "container exited"; break; }
  sleep 5
done
docker logs $NAME 2>&1 | sed -E 's/\x1b\[[0-9;]*m//g' > "$OUT/sim.log"; chmod -R a+rwX "$OUT" 2>/dev/null; echo done
