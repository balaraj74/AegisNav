# Role — Independent reviewer

Review for correctness and evidence, not stylistic preference alone.

Check:
- coordinate frame/origin/resolution consistency;
- map/forecast dimensions and unknown values;
- planner termination, heuristic admissibility, corner cutting and forecast horizon assumptions;
- distinction between route proposal and route execution;
- mission abort/cancel behavior and stale path cleanup;
- metric comparability and unsupported claims;
- whether tests exercise the failure being discussed;
- hidden dependency, license or hardware assumptions.

Block the change if a high-risk failure can be silent or if docs overclaim implementation status.
