#!/usr/bin/env bash
# Batch study: full headless Gazebo/Nav2 missions in isolated containers (no network), one per (mode, seed).
# Usage: tools/run_batch.sh "<mode> [<mode> ...]" <first_seed> <count> [parallel=2]
#   mode: sensing | sensing_notriage | omniscient | baseline. Jobs are ordered seed-major, so a partial batch
#   still has matched seeds across modes.
# Output: artifacts/batch/<mode>/seed_<n>/{metrics.json,sim.log,exit_reason.txt}. Existing finished runs are skipped.
# Each run builds the workspace in its container (~20 s) and takes ~5-15 min of wall time.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODES="${1:?modes}"; FIRST="${2:?first seed}"; COUNT="${3:?count}"; PAR="${4:-2}"
export ROOT
run_one() {
  local MODE="$1" seed="$2"
  local out="$ROOT/artifacts/batch/$MODE/seed_$seed"
  if grep -q '"done": true' "$out/metrics.json" 2>/dev/null; then echo "skip $MODE seed $seed"; return 0; fi
  mkdir -p "$out"
  local GPU=(); [ -d /dev/dri ] && GPU=(--device /dev/dri)
  echo "run  $MODE seed $seed"
  docker run --rm --init --network none --cpus "${CPUS:-3}" --memory 4g "${GPU[@]}" \
    -v "$ROOT:/repo:ro" -v "$out:/out" -e SEED="$seed" -e MODE="$MODE" -e WORLD="${WORLD:-rescue_compound}" \
    -e MAX_WALL_S="${MAX_WALL_S:-1800}" aegisnav-jazzy \
    timeout --signal=INT --kill-after=30 $(( ${MAX_WALL_S:-1800} + 120 )) bash -lc "
      set -e; source /opt/ros/jazzy/setup.bash
      mkdir -p /ws && cp -r /repo/ros2_ws/src /ws/src && cd /ws
      colcon build --symlink-install > /out/build.log 2>&1
      set +e; bash /repo/tools/docker/batch_mission.sh; chmod -R a+rwX /out" || echo "container for seed $seed exited non-zero"
}
export -f run_one
for seed in $(seq "$FIRST" $(( FIRST + COUNT - 1 ))); do for m in $MODES; do echo "$m $seed"; done; done \
  | xargs -P "$PAR" -L1 bash -c 'run_one "$0" "$1"'
