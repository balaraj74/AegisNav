#!/usr/bin/env bash
# Seeded baseline-vs-AegisNav closed-loop experiment (kinematic abstraction, no ROS/Gazebo).
# Usage: tools/run_eval.sh [trials] [showcase_seed]   (20 trials take roughly 2-3 minutes)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${ROOT}/ros2_ws/src/aegisnav_core${PYTHONPATH:+:${PYTHONPATH}}"
python3 -m aegisnav_core.experiment --trials "${1:-20}" --showcase-seed "${2:-0}" --output-dir "${ROOT}/artifacts/eval"
