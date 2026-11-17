# AegisNav: viva and hackathon Q&A

Answers are written so every statement can be backed by the repository. Where something is a limit, the honest answer is given: judges respect that more than a bluff. Batch numbers come from `docs/simulation/batch_results.md` (details and limits: `docs/simulation/batch_findings.md`).

## 1. The 30-second and 2-minute pitch

**30 seconds.** "AegisNav is a simulated rescue robot that decides *whether* to keep going, not only *where* to go. Hazards spread over time and its sensor only sees part of the building, so it plans over a forecast of the hazard, discovers new fires with its own sensor, and chooses between continue, replan, return, shelter in a refuge or abort, with a reason code for every decision. We measured it over repeated full simulated missions against a shortest-path robot."

**2 minutes.** Problem → what makes it hard (hazard spreads, the robot cannot see everything, confidence in its own sensing changes) → what the robot does (forecast, plan through time, check it can still get back, explain every decision) → what we built and tested (ROS 2 + Gazebo + Nav2, 189 unit tests, repeated missions) → what we measured (6 matched full Gazebo missions: 0/6 entered the deadly zone against 6/6 for a shortest-path robot; exposure 6.7 vs 16.8 risk·s) → what it is not (simulation only, synthetic fire model).

## 2. Problem and motivation

**What problem does it solve?** Rescue robots in a changing hazard (spreading fire or gas) must decide when a goal is no longer worth the risk. Most navigation stacks only find a path to a goal. AegisNav adds the mission decision: continue, replan, return, seek refuge or abort.

**Who is it for?** It is a research prototype for hazard-aware mission planning. It is not deployable equipment and makes no safety claim for real-world use.

**Why not just use Nav2?** We do use Nav2: AMCL for localization and its Regulated Pure Pursuit controller to follow our paths. Nav2's planners minimise distance and static costs. They do not forecast a spreading hazard, check return feasibility, or reason about perception confidence. We supply the route; Nav2 executes it and its local costmap stays authoritative for collisions.

## 3. How it works

**What is the planner?** A time-indexed risk-aware A*. A state is (cell, forecast layer). Moves cost distance plus hazard and uncertainty weighted by the forecast layers the robot would be in when it arrives. Cells above a hard threshold (0.92 in our configuration) are not entered. Travel time accumulates exactly (distance ÷ speed); an earlier version rounded every move up to a whole forecast step and overestimated times about 2.4× (ADR-0008).

**Why is the planner optimal and fast?** The heuristic is the exact cost-to-goal on a time-free relaxation (backward Dijkstra with each cell's lowest forecast cost). It never overestimates, so A* stays optimal; a test checks admissibility and consistency. It made planning on the larger map about 2× faster without changing any route cost.

**What is the forecast?** A deterministic diffusion-like rule with persistent sources. It is explicitly *not* fire or gas physics (stated in the code, ADR-0002 and the README).

**What are the five decisions?** CONTINUE_TO_TARGET, SLOW_AND_REASSESS, RETURN_TO_BASE, SEEK_REFUGE and ABORT. Each carries a machine-readable reason code and a plain explanation, shown in the operator console. The policy also checks that the route home from the target remains feasible at the predicted arrival time.

**What does "the robot discovers hazards" mean?** With `sensing:=true` the planner never sees the true hazard field. A simulated sensor reports risk only within 4 m and line of sight (walls and dense smoke block the view), with seeded noise. The robot's belief treats unseen cells as unknown (high uncertainty), not safe, and forecasts only from what it has seen.

**How is perception confidence computed?** As the smaller of a view-quality term (how much of the clear-air view is visible) and a localization term (from the AMCL position spread). It is a heuristic, **not calibrated**. A learned, calibrated detector is a planned phase, not done.

**Why hysteresis on confidence?** So the decision does not flip every cycle when confidence hovers near a threshold: after dropping below 0.55 it must recover to 0.65 before continuing. Unit-tested.

**What is "triage"?** With several inspection points, the next one is the reachable point with the lowest-exposure safe route on the current forecast; an unreachable one is set aside and retried later.

## 4. Evidence and evaluation

**How do you know it works?**
- 189 unit tests (core 133, ROS adapters 40, assets 16) plus 166 ROS package tests in a clean Jazzy container, run by CI.
- A Gazebo + Nav2 batch study: 6 matched seeds of full headless missions with seeded random fires, measured against the *real* hazard field. AegisNav with onboard sensing completed 6/6 and never entered a cell at risk ≥ 0.92 (0/6, 95% CI 0–39%); the shortest-path baseline entered one in 6/6 runs, briefly (1.5–2.0 s each), and completed 5/6. Mean exposure 6.7 ± 2.7 against 16.8 ± 6.2 risk·s; paired difference −10.1 (95% bootstrap CI −14.9 to −4.7).
- An earlier 20-seed kinematic study (corridor map): AegisNav never entered a cell at or above the hard limit; the shortest-path baseline did in every trial.

**What is the baseline?** The same planner with hazard and uncertainty weights set to 0, a hard limit of 0.99 and no mission policy (a shortest path). It answers "does hazard awareness help?", not "is Nav2's planner worse?".

**Is that a fair comparison?** Same map, same fires, same seeds, same controller. Not done yet: a reactive baseline that plans on the current hazard layer only (B1 in the research position). That is the fairest test of whether *forecasting* adds value, and we say so.

**How many runs? Is it statistically significant?** Six matched runs per mode (we planned 15 × 4 configurations and cut it for time). That is small, so we report rates with Wilson 95% intervals and a paired bootstrap interval, and we do not claim significance or a percentage improvement. Two other configurations (knows every fire; no triage) ran on only 2 seeds and are not compared.

**What failures did you see?** One run in 12 did not complete: the baseline on seed 5 stopped two cells short of the courtyard goal when the controller reported "collision ahead" three times in a row. The goal sits next to a rubble corner; that is a world/controller issue, not a hazard effect, and AegisNav could hit it too. One AegisNav run (seed 2) had a transient controller failure and recovered by replanning. In seed 5 the baseline's exposure was lower only because it did less of the mission. Earlier in development we also found and fixed: a pallet placed 0.6 m off its cells that trapped the robot, wrong travel-time accounting, a console panel that measured exposure against the robot's belief, and a planner that was too slow for the larger map.

**Why not a bigger study?** Each mission is a full simulation (about 10 minutes of wall-clock). We chose a matched, reproducible batch over a larger uncontrolled one, and the runner (`tools/run_batch.sh`) scales to more seeds.

## 5. Honest limitations (say these first)

- **Simulation only.** Gazebo, one Husky-class robot, a 2D lidar, grid planning. No real-world claim.
- **The hazard is synthetic.** The diffusion rule is configurable and is not validated physics.
- **The sensor and confidence are simulated.** Victim detection is grid line of sight, not vision. Confidence is not calibrated.
- **Position estimate:** exposure is measured at the AMCL pose, which was within 0.05–0.36 m of Gazebo's true pose (mean 0.13 m).
- **No literature search was done.** We make no novelty claim. Each component (risk-aware A*, belief maps, refuge logic) has prior art; the contribution is a tested, reproducible integration and an honest evaluation.
- **Rendering is Gazebo's own**, not photorealistic. Fire and smoke are particle effects driven by the forecast.

## 6. "Is it AI?" and "why not machine learning?"

**Is it AI?** It is classical planning and decision logic, plus a simulated sensor model. We do not call it "AI-powered". Nothing in the current system is trained. A trained perception module with measured precision, recall and calibration is the next phase (`docs/backlog/next_phase_roadmap.md`).

**Why not reinforcement learning?** The task is well-structured planning with a known cost model. A* with a forecast is exact and explainable; RL would need far more simulation and would be harder to verify against a reason code per decision.

**Is any LLM involved?** No. Adding one would make decisions less verifiable, so we avoided it.

## 7. Engineering and reproducibility

**How do I run it?** `tools/run_core_tests.sh`; `WORLD=rescue_compound SENSING=true tools/run_gui.sh demo`; `tools/run_batch.sh "sensing baseline" 1 8 3`; `python3 tools/analyze_batch.py`. Everything runs from the repository in a Docker image (`tools/docker/Dockerfile.jazzy`).

**What stops it being a mock?** The ROS nodes call the same core code, goals go to Nav2's `FollowPath` action and are cancelled on replan or abort, and the batch runs the real Gazebo/Nav2 stack. AGENTS.md forbids placeholder code on the critical path; the verification matrix states what was and was not run.

**What are the assets and licenses?** 14 CC0 Gazebo Fuel models plus one CC BY 4.0 fog texture, all pinned by hash and listed in `THIRD_PARTY_NOTICES.md`.

**What was hard?** Making the robot, controller and map agree: door widths, a 4-wheel skid-steer body that could not turn, wall clearance for a 1 m robot, particle effects that confused the lidar, and prop footprints that did not match the map. Each is recorded in an ADR with the measurement behind the fix.

## 8. Safety, ethics, impact

**Could this be used on real people?** Not in its current form. The hazard threshold is a configurable simulation rule, not a validated safety limit; real deployment would need certified sensing, validated hazard models, and human oversight. The project says so in AGENTS.md and the README.

**Who is accountable for a decision?** Every decision publishes an action, a reason code and an explanation, so a human can audit why the robot continued, held or turned back.

**What if the sensor is wrong?** The robot treats unseen cells as uncertain, not safe, and confidence drops hold or redirect the mission. But a wrong *sensor model* is exactly what the simulation cannot test; that needs real-sensor data.

## 9. Hard questions

**"Your fire isn't real, so what did you prove?"** That, *under a configurable hazard model*, planning over a forecast with an honest sensor model reduced simulated exposure compared with a shortest-path robot in 6 matched full missions (−10.1 risk·s, CI −14.9 to −4.7), and that the integrated stack runs reliably enough to measure. Not that it would rescue anyone.

**"Isn't this just A* with a cost map?"** The A* is the part we can prove optimal. The additions are the time dimension, the return-feasibility check, the belief from a limited sensor, confidence-driven decisions with reasons, and an evaluation harness that measures them in the real stack.

**"The baseline is a strawman."** It is the standard shortest path. A second honest point: it entered the deadly zone only briefly (1.5–2.0 s per run), cutting through the edge of a hazard region. The more informative comparison (a reactive hazard-cost planner) is not done yet. We list it as the next experiment.

**"What would you do with another month?"** (1) The reactive baseline; (2) a trained, calibrated camera detector; (3) collapses that really block routes; (4) a second building to test generalisation; (5) rendering on the GPU and a narrated results video.

**"Why should we trust the numbers?"** Seeds are fixed and identical across modes, measurement uses the true field not the robot's belief, failed runs are counted, and the whole pipeline is one command away.

## 10. Demo script (90 seconds)

1. (0–10 s) Overview: building, courtyard, yard, warehouse; "the robot does not know where the fires are."
2. (10–35 s) First discovery: the sensor sees a fire, the console says FIRE DETECTED, the route is cancelled and replaced. Show *Truth* beside *what the robot knows*.
3. (35–55 s) Triage: it picks the next inspection point by lowest-exposure safe route; a victim is found.
4. (55–75 s) The shortest-path robot on the same fires: enters the deadly zone.
5. (75–90 s) Results slide: the 6-seed numbers above, then the limits card.
