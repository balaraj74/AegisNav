# Rule 10 — No vibe-coded shortcuts

A polished demo is not a substitute for connected, tested behavior.

Reject changes that:
- introduce placeholder classes/functions on the mission path;
- label deterministic rules as machine learning or AI without an actual trained/inference component;
- invent results, citations, ROS topic semantics or API signatures;
- swallow failures, hard-code local paths or couple code to hidden machine state;
- implement the same planner objective in two places;
- render a hazard color without publishing a planner-consumed hazard value;
- claim path execution when only a `Path` message was published;
- make unsupported safety or novelty claims;
- add a feature without tests and a measurable acceptance criterion.

A temporary stub is allowed only when it is isolated, explicitly documented as unimplemented, excluded from success claims, and does not silently produce plausible success values.
