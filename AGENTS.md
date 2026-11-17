# AegisNav Engineering Contract for Coding Agents

This file is the canonical instruction source for any human or AI contributor. Read it before making code changes. More detailed role/rule files live in `.agent/`; they supplement this contract and may not contradict it.

## 1. Mission and scope

Build a reproducible ROS 2 simulation research system for hazard-aware rescue navigation. The core question is: **when a hazard may evolve and perception confidence may degrade, should the robot continue, replan, return to base, seek a refuge, or abort?**

The code must favor correctness, explicit assumptions, testable decisions, maintainability and honest claims over feature count or impressive-looking but disconnected components.

## 2. Source-of-truth hierarchy

When instructions conflict, use this order:
1. This `AGENTS.md` for engineering and safety constraints.
2. `docs/architecture/` for boundaries and data contracts.
3. `docs/decisions/` for accepted architecture decisions.
4. `docs/research/research_position.md` for allowed research claims and evaluation rules.
5. Issue/feature request and the existing code/tests.
6. Agent-generated assumptions, which must be labelled and confirmed before they become project facts.

Do not infer that a feature exists because it appears in a proposal, slide, TODO, image or roadmap. Confirm the implementation and test evidence.

## 3. Technical baseline

- Target platform: Ubuntu 24.04, Python 3.12, ROS 2 Jazzy and Gazebo Harmonic.
- `aegisnav_core` is simulator-independent and ROS-free. Keep its runtime to Python's standard library.
- ROS message conversions and node lifecycle belong in `aegisnav_ros`.
- `aegisnav_msgs` defines transport contracts only.
- `aegisnav_bringup` owns maps, SDF world, RViz assets and launch logic.
- Do not introduce Isaac Sim, Unreal, an LLM, a database, cloud connectivity, model downloads or another framework without a documented need and an ADR if it changes core architecture.

## 4. Design rules

1. Keep modules cohesive. Do not create a `utils.py` dumping ground, a universal manager class, or 1,000-line nodes.
2. Keep the control flow explicit. Use typed dataclasses/enums and well-defined state transitions; avoid magic strings for internal states.
3. Make domain logic pure when possible. Inputs should be passed explicitly; avoid hidden global state, random behavior without a fixed seed, implicit filesystem reads or network calls.
4. Treat the static occupancy grid, hazard risk, epistemic uncertainty and perception confidence as separate data products. Do not conflate occupied cells with dangerous-but-traversable cells.
5. Use SI units and state the time base. Document whether a field is normalized, measured, estimated, forecast or simulated.
6. Fail closed for missing map/forecast, mismatched dimensions, invalid source cells and unsafe coordinate transformations. Never silently plan against misaligned grids.
7. Every decision path needs a machine-readable reason code and a human-readable explanation.
8. Do not make the planner appear to execute a path just because it published a `nav_msgs/Path`. Action execution is a separate integration concern.
9. Never use SDF material color as the sole source of hazard risk. The planner consumes a named data topic.
10. No real-world actuation assumptions: this project is simulation research, not certified rescue equipment.

## 5. No-vibe-coding policy

The following are not acceptable:
- Replacing a requested implementation with a mock while describing it as integrated.
- Fabricating ROS topics, action names, package APIs, environment variables, outputs, benchmark results or citations.
- Swallowing broad exceptions (`except Exception: pass`) or returning success after a failed operation.
- Shipping placeholder functions on the critical path, dead code, fake AI labels, unexplained constants or TODOs with no owner/acceptance criterion.
- Adding features without a connected data path from input to decision/action and an observable validation.
- Claiming "real-time", "photorealistic", "safe", "accurate prediction", "AI-powered" or a percentage improvement without evidence.
- Disabling tests, weakening thresholds or deleting assertions only to make CI green.
- Adding unnecessary abstractions, frameworks, generated code, excessive comments that repeat syntax, or speculative architecture that the current use case does not need.
- Hardcoding a local machine's absolute path, secret, username, robot pose or network address.

## 6. Required change workflow

Before coding:
1. Inspect the current module, its callers, tests and data contracts.
2. Identify the user-visible behavior and the smallest coherent vertical slice.
3. State assumptions where the repository does not define behavior.
4. Add/update an ADR when changing an architecture boundary, ROS interface, risk semantics or runtime dependency.

While coding:
1. Implement the simplest complete path through input → core decision → output.
2. Add validation at boundaries; do not duplicate the core algorithm in ROS nodes.
3. Preserve deterministic behavior and reproducible seeds/scenarios.
4. Avoid unrelated formatting or refactors.

Before completion:
1. Run `./tools/run_core_tests.sh` and `./tools/verify_repo.sh`.
2. Run the demo and inspect `metrics.json` and SVG; do not report a result without examining the output.
3. If ROS code changed, run `colcon build --symlink-install` on a compatible ROS 2 Jazzy machine and perform the relevant topic/action smoke tests. If unavailable, say exactly that.
4. Update docs, configuration, topic tables, the verification matrix and agent rules when behavior or interfaces change.
5. Report changed areas, actual test results, unverified integration risks and how to reproduce.

## 7. Testing requirements

- Unit test validation failures and edge cases, not only the happy path.
- Planning tests must cover blocked start/goal, no path, map/forecast dimension mismatch, high-risk rejection, diagonal corner-cutting, forecast horizon behavior and deterministic output.
- Mission policy tests must cover normal continuation, degraded confidence, critical confidence, return infeasibility, refuge selection, missing forecast and no available refuge.
- ROS adapter tests must cover malformed messages, map orientation/metadata, mismatched layer counts and action cancellation on abort/replan.
- Evaluation must compare baseline and candidate using identical scenarios, map, goal and hazard source. Report distributions across multiple trials for final results; one deterministic demo is illustration, not statistical evidence.

## 8. Risk and hazard semantics

- Risk is normalized to `[0, 1]`; higher means more hazardous.
- Uncertainty/confidence are separate. In this repo, `uncertainty` is also normalized to `[0,1]`, higher means less certain.
- The current forecast uses a deterministic diffusion-like rule and persistent source values. It is not a fluid, combustion or toxicology model.
- A planner's `hard_risk_threshold` is a configurable simulation rule, not a validated human-safety threshold.
- If the forecast cannot represent the expected mission duration, flag that limitation; do not silently imply certainty beyond its horizon.
- Unknown occupancy is blocked by default in ROS adapter conversion.
- If no safe target/return/refuge route is found, publish an explicit no-safe-route/abort decision and do not send a new movement goal.
- Confidence is a simulator-supplied scalar until a perception subsystem actually estimates it. Do not call it calibrated confidence unless calibration is measured.

## 9. Dependency and security policy

- Prefer Ubuntu/ROS binary packages for the competition deadline.
- New runtime dependency requires a reason, pinned/compatible version range if applicable, license check, installation note and impact to the 16 GB RAM / 4 GB VRAM laptop.
- No cloud service, telemetry, external network call or model download in the runtime core.
- Never commit tokens, API keys, credentials, private data, user-specific absolute paths or generated environment files.
- Do not vendor third-party simulation assets until license and redistribution terms have been reviewed and recorded in `THIRD_PARTY_NOTICES.md`.

## 10. Coding style and API discipline

- Type-hint public Python functions and return values.
- Use descriptive names and explicit keyword-only options where ambiguity matters.
- Keep functions limited to one responsibility; keep ROS callbacks short and delegate algorithms to core services.
- Validate caller-provided thresholds and shapes at construction/boundaries.
- For ROS topics and actions, use standard message/action types unless a strong reason warrants a custom interface.
- Custom interfaces require comments explaining semantics, units and invalid-value conventions.
- Do not break topic names, frame ids, message layout or config parameters without updating docs and launch config.

## 11. Completion definition

A change is complete only when its behavior is implemented, failure modes are considered, tests are added, documentation matches reality, and all unrun integration steps are explicitly listed. A polished world, README or dashboard is not evidence that the planner works.
