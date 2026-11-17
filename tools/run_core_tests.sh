#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${ROOT}/ros2_ws/src/aegisnav_core:${ROOT}/ros2_ws/src/aegisnav_ros${PYTHONPATH:+:${PYTHONPATH}}"
python3 -m unittest discover -s "${ROOT}/ros2_ws/src/aegisnav_core/test" -v
python3 -m unittest discover -s "${ROOT}/ros2_ws/src/aegisnav_ros/test" -v
python3 -m unittest discover -s "${ROOT}/ros2_ws/src/aegisnav_bringup/test" -v
