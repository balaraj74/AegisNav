# Try it: what to test and what you should see

Start (this laptop, via the container):
- `tools/run_gui.sh sandbox`: the robot runs the rescue mission on its own (victim 1 → victim 2 → base). Nothing else is scripted: **you** cause every fire, confidence change and new goal.
- `tools/run_gui.sh demo`: plays the fixed script instead (good for recording, not for testing):
  - at 8 s burning debris blocks the hall beyond the west door, forcing a detour;
  - perception confidence drops to 0.42 at 105 s, on the way back to base.

Open the **operator console** at http://localhost:8080 (any browser on this machine). It shows:
- the live map with the AegisNav route and the hazard-ignoring shortest route, both scored on the same forecast;
- the decision and its reason;
- the mission progress and an event log.

The Gazebo window is the 3D view, with a camera that follows the robot and cuts to new fires. Add `RVIZ=true` to also open RViz.

## Rescue building (default world)

Inputs (all simulated):
- console: **Click: start fire** / **Click: set goal** on the map, and the **Perception confidence** slider;
- terminal: `tools/sim_cmd.sh fire X Y`, `goal X Y`, `confidence V`, `status`.

Landmarks (metres, map frame):

| Landmark | Position |
|---|---|
| Base / start (blue) | −8.25, −5.25 |
| Victim 1 (approach point) | 3.75, 5.25 |
| Victim 2 (approach point) | 8.25, −5.25 |
| Refuges (green) | −8.25, 5.25 · −1.75, −5.25 · 7.75, 5.25 |
| Permanent fire source | −0.25, −0.25 |
| Gas leak | 0.25, −3.75 |
| Scripted ignitions (demo only) | −7.25, 3.75 · −6.75, 3.75 |

Tests (tests 1 and 2 were observed in this world in headless and Xvfb-GUI runs in the container on 2026-10-29; test 3 so far only in the corridor world; none yet on the laptop):

1. **Mission.** Just watch.
   - Expect the robot to go to victim 1, then victim 2, then back to base.
   - The console's mission panel ticks off each victim.
   - The decision stays `CONTINUE_TO_TARGET | mission_feasible` while the route is clear.
2. **It reacts to a fire you place.** With **Click: start fire**, click on the cyan route a few metres ahead of the robot.
   - Expect within one or two planning cycles (about 1–3 s) the event log to show `FIRE at (…)` → `Route cancelled (replan_cancel)` → `New route dispatched`.
   - The old route stays as a grey ghost and the new route goes around the fire.
   - The shortest-path comparison now shows `CROSSES HARD LIMIT` once the fire's risk on it reaches 0.92.
3. **Perception degrades.** Drag the slider below 0.30, e.g. 0.2.
   - Expect `SEEK_REFUGE | perception_confidence_critical`, and the robot drives to a reachable refuge.
   - Between 0.30 and 0.55 you should get `SLOW_AND_REASSESS | perception_confidence_degraded`.
4. **What must NOT happen:**
   - the robot driving through red tiles;
   - the old route continuing after `Route cancelled`;
   - a new route being sent after an `ABORT`.

## Rescue compound (larger inspection area): `WORLD=rescue_compound SENSING=true tools/run_gui.sh demo`

36 × 22 m: the rescue building plus a courtyard (north), a yard (east) and a warehouse with shelving aisles (north-east).

With `SENSING=true` the robot does **not** know where the hazards are:
- it learns them from a simulated onboard sensor (4 m range, walls and dense smoke block the view);
- it picks the next of five inspection points by the lowest-exposure safe route (triage);
- its perception confidence is computed from how much it can see and how well it is localized.

The console's **Truth** button shows the real field the robot cannot see.

| Landmark | Position (metres, map frame) |
|---|---|
| Base / start | −8.25, −5.25 |
| Inspection points | 3.75, 5.25 · 8.25, −5.25 · 20.25, −4.25 (yard) · 24.25, 13.25 (warehouse) · 6.75, 13.25 (courtyard) |
| Hazard sources (unknown to the robot) | fire −0.25, −0.25 · gas 0.25, −3.75 · warehouse fire 19.25, 8.25 · yard fire 15.25, −0.75 |
| Refuges | −8.25, 5.25 · −1.75, −5.25 · 7.75, 5.25 · 24.25, −5.25 · −7.25, 13.25 |

What you should see (observed headless in the container on 2026-10-30; not yet on the laptop):
1. **Discovery and reroute:**
   - event log `FIRE DETECTED by robot sensor` (gas leak at about 18 s, corridor fire at about 22 s);
   - then `Route cancelled` and a new route;
   - the camera cuts to each discovered fire.
2. **Inspection:**
   - `VICTIM FOUND by robot sensor` as each of the four victims comes into view;
   - points 1 → 2 → yard → warehouse → courtyard, then base (back at about 7 min wall-clock);
   - the yard and warehouse fires are discovered on the way and avoided.
3. **Your tests:**
   - **Fire on the route:** click **Click: start fire** on the route ahead. Under sensing, the robot reacts only once the fire is within its 4 m line of sight. Compare "Hazard now" with **Truth**.
   - **Unreachable point:** surround an inspection point with fires. Expect the log `Triage (no safe route ...) set aside` and a switch to another point. *Not yet verified live.*
4. **What must NOT happen:**
   - the robot reacting to a fire it cannot see yet;
   - the robot entering a red tile;
   - a stalled robot while the 3D view shows it pushing against an object (report it: that would be a prop footprint bug).

## Corridor world (`WORLD=corridor tools/run_gui.sh sandbox`)

Inputs: as in the rescue building above, or RViz's **2D Goal Pose** and **Publish Point** (click = fire) with `RVIZ=true`.

Map landmarks (metres):
- base (blue) −3.8, −2.2;
- target (gold) 3.8, 2.2;
- refuges (green) −3.8, 1.8 / −2.2, 1.8 / 3.8, −2.2;
- upper gate −0.2, 1.4–1.8;
- lower gate −0.2, −1.8 (already burning) and −0.2, −1.4.

### Corridor sandbox tests
Each expected result below was observed in headless runs on 2026-10-29 (`tools/docker/sandbox_test.sh`, `sandbox_block_test.sh`).

1. **Basic navigation.** Just watch after launch (or set any goal with 2D Goal Pose / `goal X Y`; your goal replaces the default one).
   Expect for the gold target: overlay `CONTINUE_TO_TARGET | mission_feasible`; cyan route through the **upper** gate (the lower gate is on fire); robot drives along it.
2. **It reacts to a fire you place** (the "is this hardcoded?" test). While it drives, Publish Point on the cyan route ahead of the robot (e.g. `fire 2.2 2.2`).
   Expect within ~2–3 s: event log `FIRE started` → `route CANCELLED (replan_cancel)` → `new route sent`; a **grey ghost** of the old route and a **cyan** new route that goes around the fire; the robot keeps going to the same target. Click somewhere else next time — the detour changes with where you click.
3. **Block every way through.** With the target as goal (the default), fires at `-0.2 -1.4`, `-0.2 1.4`, `-0.2 1.8`.
   Expect: `SEEK_REFUGE | target_route_unavailable`; robot drives to a reachable green refuge instead of entering fire.
4. **Perception degrades.** `confidence 0.2` → `SEEK_REFUGE | perception_confidence_critical`, robot goes to the nearest refuge. A value between 0.30 and 0.55 (e.g. `confidence 0.34`) → `SLOW_AND_REASSESS | perception_confidence_degraded`.
5. **What must NOT happen:** the robot driving into red tiles; the old route continuing after `route CANCELLED`; a new route being sent after an `ABORT`.

## Notes
- Fire, risk and confidence are a simulated model (diffusion-like spread), not real fire physics or a perception estimate. Tiles are drawn at display thresholds (risk ≥ 0.25/0.5/0.75); the planner's hard limit is 0.92 and it also weighs lower risk.
- Fires persist and keep spreading (persistent sources, slow decay): after about 2 minutes most of the arena is orange/red — that is the synthetic model, not a display fault. Restart to reset. Gazebo tiles refresh every 2 s, RViz every 1 s, the console 4 times per second.
- Rendering: NVIDIA GPU if the NVIDIA container runtime is installed, otherwise the integrated Intel GPU through `/dev/dri` (hardware, via Mesa); software only if neither is available.
