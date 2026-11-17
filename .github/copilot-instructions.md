# GitHub Copilot instructions for AegisNav

Read and obey [`../AGENTS.md`](../AGENTS.md). The simulator-independent core in `ros2_ws/src/aegisnav_core` must remain standard-library-only and ROS-free. ROS interfaces live in `aegisnav_msgs`; conversions and nodes live in `aegisnav_ros`; simulator assets and launches live in `aegisnav_bringup`.

No fake integration, fabricated benchmark, unsupported novelty claim, hidden cloud request, silent fallback or safety claim is allowed. Keep hazard risk, uncertainty and perception confidence separate. Fail closed on malformed maps/forecasts. Add regression tests and update architecture/data-contract docs for interface changes. Before finishing, run `./tools/run_core_tests.sh`, `./tools/run_demo.sh`, and `./tools/verify_repo.sh`; if ROS code changed, explicitly state whether a real ROS 2 Jazzy/Gazebo Harmonic build and runtime smoke test were executed.
