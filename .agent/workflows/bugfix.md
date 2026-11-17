# Workflow — Bug fix

1. Reproduce the bug or state clearly why it cannot be reproduced.
2. Identify the violated contract and write a minimal failing test if feasible.
3. Fix the cause at the correct layer; avoid compensating hacks in callers.
4. Add regression coverage including nearby edge cases.
5. Run the relevant tests and all core quality gates.
6. Update docs if externally visible behavior or troubleshooting steps changed.
7. Report prior failure mode, root cause, fix, and evidence.
