#!/usr/bin/env bash
# Inside the Jazzy container: run the full seed scenario headless, add two observer cameras
# (not part of the robot or the planning world), and save PNG frames to /out.
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
WORLD=aegisnav_rescue_world
# Virtual display for RViz (software OpenGL); Gazebo itself runs server-only.
Xvfb :99 -screen 0 1400x900x24 > /tmp/xvfb.log 2>&1 &
export DISPLAY=:99
sleep 2
ros2 launch aegisnav_bringup sim_world.launch.py headless:=true start_rviz:=true seed:=${SEED:-7} > /tmp/sim.log 2>&1 &
( for i in 1 2 3 4 5 6; do sleep ${RVIZ_EVERY:-12}; import -window root /out/rviz_$(printf %02d $i).png 2>/dev/null; done ) &
sleep 14
cam() {  # name x y z roll pitch yaw
  local sdf="<sdf version='1.9'><model name='$1'><static>true</static><pose>$2 $3 $4 $5 $6 $7</pose><link name='l'><sensor name='c' type='camera'><always_on>1</always_on><update_rate>2</update_rate><topic>/aegisnav/$1</topic><camera><horizontal_fov>1.05</horizontal_fov><image><width>1280</width><height>720</height><format>R8G8B8</format></image><clip><near>0.1</near><far>40</far></clip></camera></sensor></link></model></sdf>"
  gz service -s /world/$WORLD/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req "sdf: \"$sdf\"" >/dev/null
}
cam overview_camera 0 -7.6 8.2 0 0.92 1.5708
cam topdown_camera 0 0 9.5 0 1.5708 1.5708
ros2 run ros_gz_bridge parameter_bridge /aegisnav/overview_camera@sensor_msgs/msg/Image[gz.msgs.Image /aegisnav/topdown_camera@sensor_msgs/msg/Image[gz.msgs.Image > /tmp/cambridge.log 2>&1 &
python3 /repo/tools/docker/capture_frames.py /out /aegisnav/overview_camera:overview /aegisnav/topdown_camera:topdown --at ${AT:-2,22,40,60}
sed -E 's/\x1b\[[0-9;]*m//g' /tmp/sim.log | grep -E "Decision|ignition|Reached|Cancelling|tile" | cut -c1-160 > /out/events.txt
cp /tmp/sim.log /out/sim.log
