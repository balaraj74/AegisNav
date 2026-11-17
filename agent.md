# Agent entrypoint

**Read [`AGENTS.md`](AGENTS.md) first.** It is the authoritative engineering contract and overrides any generic coding-agent defaults.

Before implementation, read:
- `.agent/README.md`
- `.agent/rules/` rules relevant to the task
- `docs/architecture/system_overview.md` and `docs/architecture/data_contracts.md`
- `docs/research/research_position.md` before making novelty or performance claims

Do not describe stubs as working integrations. Preserve the ROS-free core, test meaningful behavior, keep hazard/uncertainty/confidence distinct, fail closed on malformed geometry/forecast, and report unverified ROS/Gazebo steps explicitly.
