# Engineering standards

## Maintainability

- Keep modules focused and public APIs typed.
- Avoid global mutable state and hidden configuration.
- Use immutable dataclasses for domain facts, enums for mission actions, and explicit config objects for thresholds/weights.
- Do not create generic frameworks for only one use case.
- Keep time, coordinate, normalized risk and confidence semantics in docs and names.
- Keep simulator/ROS APIs out of the pure core.

## Error handling

- Raise clear exceptions for invalid shapes, out-of-range cells and mismatched grids.
- ROS nodes must catch expected input-validation exceptions, log a specific message and refrain from publishing a misleading route.
- Do not catch every exception unless a boundary genuinely must keep a process alive; log details and preserve observability.
- When a no-safe-route decision occurs, clear the published path and cancel an active goal if opt-in actuation is running.

## Algorithmic quality

- Keep edge costs non-negative.
- Preserve the planner heuristic's admissibility relative to the move model.
- Prevent diagonal corner-cutting.
- Test the risk cutoff separately from soft objective weights.
- Document the forecast tail behavior and mission time horizon.
- Add benchmark artifacts for any change to cost weights or forecast behavior.

## Configuration management

- No hidden default should override the checked-in YAML silently.
- Keep ROS parameter names aligned across setup, docs and code.
- The sample grid has exact source/base/refuge cells; changing the map requires changing those cells and testing the alignment.
- Secrets and per-machine paths do not belong in committed files.

## Git workflow

- Use small commits with a behavior-focused message.
- Avoid committing local ROS build directories (`build/`, `install/`, `log/`) or simulator caches.
- Commit deterministic test artifacts only when they are intentional reference baselines; generated `artifacts/demo/` are ignored by default.
- Do not merge unless unit tests pass and any skipped integration test is recorded.
