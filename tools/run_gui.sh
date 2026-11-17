#!/usr/bin/env bash
# Run the full AegisNav simulation WITH GUI (Gazebo + RViz) in the aegisnav-jazzy container,
# for hosts without a native ROS 2 Jazzy install (e.g. Ubuntu 26.04).
# Usage: tools/run_gui.sh sandbox        # robot runs the mission; YOU add fires, confidence, new goals
#        tools/run_gui.sh demo           # scripted events (fire in the corridor, later sensor degradation)
# Env: WORLD=rescue_building|rescue_compound|corridor (default rescue_building), RVIZ=true to also open RViz,
#      SENSING=true: the robot only knows hazards its simulated onboard sensor has seen.
# The operator console is served at http://localhost:8080.
# Test card: docs/operations/try_it.md (also printed below).
#
# Display: X11 socket (works under Wayland via XWayland). Access is granted only to the
# container's root user for the duration of the run and revoked on exit.
# GPU: NVIDIA if the NVIDIA container runtime is installed; otherwise the integrated GPU through
# /dev/dri (Mesa); software rendering only if neither is available.
# Stop: Ctrl-C in this terminal (the container is removed on exit).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-sandbox}"
SEED="${2:-7}"
case "$MODE" in
  sandbox) SCRIPTED=false ;;  # robot drives to the target; fires/confidence/goals come from you
  demo) SCRIPTED=true ;;
  *) echo "usage: $0 [sandbox|demo [seed]]" >&2; exit 2 ;;
esac
if ! [[ "$SEED" =~ ^[0-9]+$ ]]; then
  echo "seed must be a non-negative integer" >&2
  exit 2
fi
if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set; run this from a desktop session." >&2
  exit 2
fi
if ! docker image inspect aegisnav-jazzy >/dev/null 2>&1; then
  echo "Image aegisnav-jazzy not found. Build it first:" >&2
  echo "  docker build -f tools/docker/Dockerfile.jazzy -t aegisnav-jazzy tools/docker" >&2
  exit 2
fi

GPU_ARGS=()
if docker info 2>/dev/null | grep -qi nvidia; then
  GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
  echo "NVIDIA container runtime found: rendering on the NVIDIA GPU."
elif [ -d /dev/dri ]; then
  # Intel/AMD GPUs work through Mesa with just the DRM device (verified: Intel Iris Xe "ADL GT2").
  GPU_ARGS=(--device /dev/dri)
  echo "Rendering on the integrated GPU via /dev/dri (Mesa)."
else
  echo "No GPU device available: software rendering (expect a low frame rate)."
fi

case "${WORLD:-rescue_building}" in
  corridor) SECTION='^## Corridor world' ;;
  rescue_compound) SECTION='^## Rescue compound' ;;
  *) SECTION='^## Rescue building' ;;
esac
sed -n "/${SECTION}/,/^## /p" "${ROOT}/docs/operations/try_it.md" | sed '$d'
xhost +SI:localuser:root >/dev/null
trap 'xhost -SI:localuser:root >/dev/null' EXIT

echo "Operator console: http://localhost:8080  (opens once the simulation is up, ~40 s)"
# Bridge network only so the console port can be published to *localhost*; ROS discovery stays
# inside the container (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST).
docker run --rm -it --init --name aegisnav_gui -p 127.0.0.1:8080:8080 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${ROOT}:/repo:ro" \
  "${GPU_ARGS[@]}" \
  aegisnav-jazzy bash -lc "
    set -e
    source /opt/ros/jazzy/setup.bash
    mkdir -p /ws && cp -r /repo/ros2_ws/src /ws/src && cd /ws
    echo 'Building workspace...'
    colcon build --symlink-install >/dev/null
    source install/setup.bash
    ros2 launch aegisnav_bringup sim_world.launch.py world:=${WORLD:-rescue_building} seed:=${SEED} scenario:=true \
      scripted_events:=${SCRIPTED} start_rviz:=${RVIZ:-false} sensing:=${SENSING:-false}"
