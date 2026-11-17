#!/usr/bin/env bash
# Inside the Jazzy container: full launch WITH the Gazebo GUI on a virtual display (Xvfb) and the
# camera director, screenshots of the whole screen to /out every ${EVERY:-12} s. Verification aid.
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
Xvfb :99 -screen 0 1600x900x24 > /tmp/xvfb.log 2>&1 &
export DISPLAY=:99
sleep 2
ros2 launch aegisnav_bringup sim_world.launch.py world:=${WORLD:-rescue_building} > /out/sim.log 2>&1 &
for i in $(seq 1 ${SHOTS:-10}); do sleep ${EVERY:-12}; import -window root /out/gui_$(printf %02d $i).png 2>/dev/null; done
sed -E 's/\x1b\[[0-9;]*m//g' /out/sim.log | grep -E "Camera:|Decision |ignition|Goal reached|patience|Stale" | cut -c1-140 > /out/events.txt
