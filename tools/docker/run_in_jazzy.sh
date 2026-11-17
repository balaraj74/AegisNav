#!/usr/bin/env bash
# Run a command inside the aegisnav-jazzy image with the repo mounted read-only at /repo.
# The workspace is copied to /ws in the container so build artefacts never touch the source tree.
# Isolation: --network none gives each run its own ROS graph (no DDS cross-talk with other
# containers); the timeout runs *inside* the container so it cannot outlive the run.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GPU=()
[ -d /dev/dri ] && GPU=(--device /dev/dri)  # integrated GPU (Mesa) for sensor/camera rendering when present
docker run --rm --init --network none "${GPU[@]}" -v "${ROOT}:/repo:ro" ${AEGIS_DOCKER_ARGS:-} aegisnav-jazzy \
  timeout --signal=INT --kill-after=20 "${AEGIS_TIMEOUT_S:-540}" bash -lc "
  set -e; source /opt/ros/jazzy/setup.bash
  mkdir -p /ws && cp -r /repo/ros2_ws/src /ws/src && cd /ws
  $*"
