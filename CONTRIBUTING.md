# Contributing to AegisNav

AegisNav is a simulation research starter. Contributions should improve an observable behavior, a validated contract, the reproducibility of experiments, or the clarity of evidence. Feature count and polished output are not sufficient.

## Required read order

1. [`AGENTS.md`](AGENTS.md) — canonical engineering contract.
2. `.agent/rules/` — focused rules for source-of-truth, quality, ROS and dependencies.
3. Relevant role and workflow file in `.agent/roles/` and `.agent/workflows/`.
4. The applicable architecture document and accepted ADR.
5. Research-positioning rules before making claims about novelty or performance.

## Change workflow

1. Describe the behavior, failure mode or hypothesis in one paragraph.
2. Identify the boundary being changed: domain core, ROS adapter, custom message, simulation/bringup, or evaluation.
3. Add an ADR before changing public topic/message contracts, planner semantics, dependencies or package boundaries.
4. Add a failing test for a bug or acceptance test for a feature before or alongside implementation.
5. Keep the pure-Python core deterministic, ROS-free, network-free and standard-library-only.
6. Run `./tools/run_core_tests.sh`, `./tools/run_demo.sh`, and `./tools/verify_repo.sh`.
7. For ROS changes, additionally build with `colcon build --symlink-install` and run the corresponding live ROS/Gazebo smoke test on Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic. If unavailable, identify it as unverified.
8. Update the data-contract docs, verification matrix, configuration and runbook where applicable.
9. Report changed files, actual command results, unverified work, remaining risks and reproduction steps.

## Pull request acceptance checklist

- [ ] The behavior is connected end-to-end; no critical-path stub is described as implemented.
- [ ] Boundary inputs, invalid data and failure states are handled explicitly.
- [ ] Tests cover success and relevant failure/edge cases.
- [ ] The baseline and candidate receive equivalent map, goal and forecast inputs.
- [ ] Claims are backed by captured output; no extrapolated or invented performance result.
- [ ] No secrets, personal absolute paths, unlicensed assets or unnecessary dependencies were added.
- [ ] Required docs, agent instructions and third-party notices are updated.
- [ ] Test and integration status is reported honestly.
