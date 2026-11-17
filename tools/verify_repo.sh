#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"${ROOT}/tools/run_core_tests.sh"
PYTHONPATH="${ROOT}/ros2_ws/src/aegisnav_core${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m compileall -q "${ROOT}/ros2_ws/src"
python3 "${ROOT}/tools/verify_repo.py"
"${ROOT}/tools/run_demo.sh"
printf '\nAll local source-only quality gates passed. This does not replace ROS/Gazebo integration testing.\n'
