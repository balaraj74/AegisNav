# AegisNav agent operating system

`AGENTS.md` is the canonical contract. This directory adds focused reusable rules and role guidance for agents that plan, implement, review, research, or integrate the simulation.

## Read order

1. Root `AGENTS.md`.
2. `rules/00-source-of-truth.md` and `rules/10-no-vibecoding.md`.
3. The role file matching the current task.
4. The workflow file matching the change type.
5. Relevant architecture and safety docs.

## Expected work report

Every coding task must end with: (a) what changed, (b) exact commands/tests run and observed results, (c) what was not tested, (d) assumptions or risks, and (e) a reproduction command. Do not claim work was run if it was only reasoned about.
