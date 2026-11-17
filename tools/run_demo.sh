#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${ROOT}/ros2_ws/src/aegisnav_core${PYTHONPATH:+:${PYTHONPATH}}"
python3 -m aegisnav_core.cli --output-dir "${ROOT}/artifacts/demo"
printf '\nGenerated artifacts:\n  %s\n  %s\n' \
  "${ROOT}/artifacts/demo/aegisnav_route_comparison.svg" \
  "${ROOT}/artifacts/demo/metrics.json"
