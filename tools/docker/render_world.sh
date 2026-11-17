#!/usr/bin/env bash
# Inside the Jazzy container: load a world headless, add observer cameras, save PNGs to /out.
# Usage: render_world.sh <world.sdf path> ; cameras: CAMS="name:x y z roll pitch yaw;..."
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix aegisnav_bringup)/share/aegisnav_bringup/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}
gz sim -r -s --headless-rendering -v 2 "$1" > /tmp/gz.log 2>&1 &
sleep 15
IFS=';' read -ra LIST <<< "${CAMS}"
TOPICS=(); ARGS=()
for entry in "${LIST[@]}"; do
  name=${entry%%:*}; pose=${entry#*:}
  sdf="<sdf version='1.9'><model name='$name'><static>true</static><pose>$pose</pose><link name='l'><sensor name='c' type='camera'><always_on>1</always_on><update_rate>2</update_rate><topic>/obs/$name</topic><camera><horizontal_fov>1.1</horizontal_fov><image><width>1280</width><height>720</height><format>R8G8B8</format></image><clip><near>0.1</near><far>120</far></clip></camera></sensor></link></model></sdf>"
  gz service -s /world/aegisnav_rescue_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req "sdf: \"$sdf\"" >/dev/null
  TOPICS+=("/obs/$name@sensor_msgs/msg/Image[gz.msgs.Image"); ARGS+=("/obs/$name:$name")
done
ros2 run ros_gz_bridge parameter_bridge "${TOPICS[@]}" > /tmp/br.log 2>&1 &
python3 /repo/tools/docker/capture_frames.py /out "${ARGS[@]}" --at 3
grep -iE "error|unable|fail" /tmp/gz.log | grep -v "pci id\|dri2\|getifaddrs" | head -8
