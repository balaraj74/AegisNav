# Workflow — Feature delivery

1. **Read:** root contract, relevant architecture docs, module and existing tests.
2. **Specify:** user-facing behavior, data contracts, acceptance criteria, invalid inputs and safe fallback.
3. **Design:** choose the smallest vertical slice; document cross-package changes in an ADR if needed.
4. **Implement:** core behavior first; adapt it to ROS only after core semantics are tested.
5. **Test:** regression tests, invalid input tests, deterministic demo and repository checks.
6. **Integrate:** if ROS/Gazebo touched, build on a compatible target and verify topic/action behavior; do not infer from syntax checks.
7. **Document:** update README/topic tables/config/verification matrix/roadmap.
8. **Report:** describe actual changes, observed commands and outputs, gaps, risks and how to reproduce.
