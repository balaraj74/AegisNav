#!/usr/bin/env bash
# Inside the Jazzy container: one headless mission for the batch study.
# Env: WORLD (rescue_compound), SEED, MODE (sensing | sensing_notriage | omniscient | baseline), MAX_WALL_S, MAX_MISSION_S.
# Output (in /out): metrics.json (mission recorder), sim.log (filtered), exit_reason.txt
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
WORLD=${WORLD:-rescue_compound}; SEED=${SEED:?}; MODE=${MODE:?}
case "$MODE" in
  sensing)          ARGS="sensing:=true" ;;
  sensing_notriage) ARGS="sensing:=true triage:=false" ;;
  omniscient)       ARGS="sensing:=false" ;;
  baseline)         ARGS="sensing:=false baseline:=true" ;;
  *) echo "unknown MODE $MODE" >&2; exit 2 ;;
esac
ros2 launch aegisnav_bringup sim_world.launch.py headless:=true world:=$WORLD seed:=$SEED ignition_seed:=$SEED \
  scenario:=true scripted_events:=true console:=false visualization:=false start_rviz:=false \
  record_path:=/out/metrics.json max_mission_s:=${MAX_MISSION_S:-900} slow_cycle_warn_s:=0.0 $ARGS > /tmp/sim.log 2>&1 &
LAUNCH=$!
START=$(date +%s); REASON=wall_timeout
while [ $(( $(date +%s) - START )) -lt ${MAX_WALL_S:-1800} ]; do
  sleep 5
  if grep -q '"done": true' /out/metrics.json 2>/dev/null; then REASON=recorder_done; break; fi
  kill -0 $LAUNCH 2>/dev/null || { REASON=launch_exited; break; }
done
echo "$REASON wall_s=$(( $(date +%s) - START ))" > /out/exit_reason.txt
kill -INT $LAUNCH 2>/dev/null; sleep 6
sed -E 's/\x1b\[[0-9;]*m//g' /tmp/sim.log | grep -E "Decision |ignition accepted|discovered|Victim detected|Triage|Cancelling F|Reached the goal|FollowPath terminated|patience|collision ahead|Stale|rejected|Traceback|Error|Planning cycle took|First pose|Mission recorder" > /out/sim.log
