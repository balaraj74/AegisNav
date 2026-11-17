# Competition demonstration runbook

## Story in 90–110 seconds

1. **Scene and mission (0–15 s):** show the robot, rescue target, map and source hazard markers; state that the robot plans under changing risk, not just distance.
2. **Baseline (15–30 s):** show the distance-only route crossing the high-risk gate. Use the deterministic SVG or a separate baseline run.
3. **AegisNav route (30–50 s):** show the risk-aware alternate route and the forecast heatmap; surface the `reason_code` and exposure metric.
4. **Perception degradation (50–70 s):** publish a lower confidence value, show `slow_and_reassess`/refuge selection and explain that the score is currently a simulated input.
5. **Return feasibility (70–90 s):** show the route-to-target and return route assessment; introduce a hazard that makes the future return route infeasible and show a refuge/abort decision.
6. **Evidence (90–110 s):** finish with the same-scenario baseline comparison and the metrics file. Do not quote an unmeasured percentage.

Keep the primary demo stable. Dynamic changes can be scripted via ROS parameters/topic inputs or by a deterministic test harness. Do not improvise risky changes live in front of judges.

## Capture sequence

- Use 720p first; verify frame rate and readability on the actual laptop.
- Record one clean Gazebo window and one RViz view if you cannot integrate overlays into Gazebo.
- Capture the SVG/metrics separately for a short cut or overlay, but label it as a computed planning result rather than a Gazebo camera view.
- Save the raw recording and the rendered 1–2 minute file to local storage, then upload before the organizer's deadline.
- Test the shared link from a private/incognito session and verify permission.
- Keep an offline copy of the video and a core-demo fallback. Do not rely on a first-time Fuel download at the venue.

## Day-of checklist

- [ ] Laptop power adapter and enough free disk space.
- [ ] ROS 2 Jazzy shell sources exactly once; no Humble/Jazzy mix.
- [ ] Run core tests and `verify_repo.sh`.
- [ ] Launch cached simulation world; confirm map/forecast/path topics.
- [ ] Disable optional FollowPath actuation unless already verified end to end.
- [ ] Warm up Gazebo and recording software; close unrelated GPU applications.
- [ ] Check external-drive/cloud permission and retain local backup.
- [ ] Explain limitations honestly if a component is visual-only or a confidence input is simulated.

## Source-only closed-loop demo (works without ROS)

```bash
./tools/run_core_tests.sh
./tools/run_eval.sh 20 0                       # study + seed-0 trace
PYTHONPATH=ros2_ws/src/aegisnav_core python3 -m aegisnav_core.experiment --trials 1 --first-seed 7 --showcase-seed 7 --output-dir artifacts/showcase_seed7
```
Seed 7 ignites near the upper gate at t=16 s; the trace shows `continue_to_target` -> `seek_refuge` (`target_route_unavailable`) with cancel then replacement dispatch. These outputs are not Gazebo recordings. The Gazebo/Nav2 demo (above) has not been executed; no screenshot or video of it exists.

Decision timelines (`*_timeline.svg`) are written next to each showcase trace by `aegisnav_core.experiment`: colour = action, label = reason code, markers = ignition / confidence change / cancel / send. They are rendered from the simulated trace, not from a Gazebo run.

## Recorded-demo sequence (verified headless; rehearse with GUI before recording)
1. `ros2 launch aegisnav_bringup sim_world.launch.py seed:=7 start_rviz:=true` (native 24.04) or `tools/run_gui.sh 7` (container, any host with Docker), and wait for "Decision continue_to_target".
2. ~16 s after the first pose: "Hazard ignition accepted at cell (10, 12)" then "Cancelling FollowPath goal (replan_cancel)" — the route changes.
3. ~28 s: "Decision slow_and_reassess (perception_confidence_degraded)" — robot heads to refuge (21, 2).
4. "Reached the goal!" at the refuge.
5. For metrics alongside: `./tools/run_eval.sh 20 0` output in `artifacts/eval/` (kinematic runner, labelled as such) and the seed-7 timeline `artifacts/showcase_seed7/*_timeline.svg`.
Fallback if Gazebo misbehaves on the laptop: `headless:=true` plus the kinematic showcase artifacts; say on screen that they are not Gazebo output.

## Recording the rescue-building demo (Gazebo GUI + operator console)

**Layout.** `tools/run_gui.sh demo` opens Gazebo with the recording layout: the 3D view plus small play/stats overlays, without editor panels. The camera director chooses the shots:
- 6 s overview, then chase;
- 5 s cut-away to each new fire;
- 5 s top-down view on each change of decision action.

Open http://localhost:8080 in a browser next to Gazebo and press **Presentation**. It hides the click-mode controls and the lower side panels, and enlarges the decision caption. Recommended split on a 1920×1080 screen: Gazebo left 60 %, console right 40 %.

**Recorder.** Use the desktop's own screen recorder or OBS, already installed (no new dependency). Record at 1080p30 and do a 30 s test first, checking frame rate and readability.

**What the script shows** (timestamps are mission time from the first localized pose; observed in container runs, rehearse on the laptop):
1. The robot leaves base for victim 1 up the west corridor. The console decision reads `CONTINUE_TO_TARGET`, and the AegisNav and shortest routes are drawn together.
2. At 8 s burning debris appears across the hall beyond the west door, on the planned route. The director cuts to it. About 3 s later the route is cancelled, and a replacement goes through the middle room, keeping clear of the fire. The shortest-path comparison shows the risk being avoided.
3. Victim 1 is reached (~58 s), then victim 2 (~97 s), and the mission panel updates.
4. At 105 s on the return leg, simulated confidence drops to 0.42. The decision becomes `SLOW_AND_REASSESS (perception_confidence_degraded)`: the route home is cancelled and the robot stops at a reachable low-risk refuge instead of continuing blind.

**Say on screen or in narration:**
- the hazard field is a deterministic synthetic forecast, not fire physics;
- confidence is a simulated input;
- the shortest-path line is the planner's own comparison on the same forecast, not a separate baseline robot run.

Do not quote a percentage improvement from one run.
