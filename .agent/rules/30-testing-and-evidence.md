# Rule 30 — Tests and evidence

- Add a regression test for every behavior change and every fixed defect.
- Test a negative/invalid case at the same boundary as the happy path.
- Run `./tools/run_core_tests.sh` and `./tools/verify_repo.sh` after changes.
- Run `./tools/run_demo.sh`, inspect the generated SVG and JSON, and do not state metrics that were not observed.
- ROS integration requires a real Jazzy build plus `ros2 topic info/echo`, map/forecast dimension checks, action-server verification, route replan, and abort cancellation tests.
- A single demo scenario is illustrative. Final competition claims should be supported with repeated controlled trials and an explicit baseline.
