# Rule 00 — Source of truth

- Treat root `AGENTS.md` as authoritative.
- Treat architecture docs and ADRs as contracts, not optional prose.
- Distinguish implemented behavior from proposed behavior, TODOs and visual props.
- Never infer runtime compatibility from a package name alone; confirm dependencies and run a target-environment smoke test when possible.
- When documentation and code disagree, report the discrepancy and update both in the same change.
