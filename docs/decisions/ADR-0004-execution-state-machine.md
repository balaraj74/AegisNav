# ADR-0004: Pure execution state machine and shared decision-to-route mapping

- **Status:** Accepted (core tested; ROS wiring not yet run against Nav2)
- **Context:** ADR-0003 requires that a published path is not an executed path and that cancel/replan are safe. The previous node tracked a single goal handle and a route signature inline, could send a new goal while the old one was still running, and duplicated the action-to-route selection that the evaluation also needs.
- **Decision:**
  1. `aegisnav_core.execution.ExecutionController` (states IDLE, DISPATCHING, EXECUTING, CANCELLING, FAULT) decides SEND/CANCEL/NONE from the desired route and from goal-response/result events. At most one goal is in flight; a replacement is sent only after termination of the old goal; stopping drops any pending replacement; repeated failures latch FAULT.
  2. `MissionAssessment.execution_route` is the single definition of which route a decision executes (ABORT never yields one).
  3. `stale_inputs` fails closed when pose or forecast are missing/old; `planner_node` publishes an explicit abort decision, clears the path and cancels any goal.
  4. `aegisnav_core.simulation` runs the whole loop with a kinematic stand-in action server and measures ground-truth exposure.
- **Consequences:** Dispatch logic is testable without ROS. The ROS node became a thin translator, but that translation (action-client futures, GoalStatus mapping, threading) is unverified until built and run on Jazzy with Nav2. The kinematic runner is an abstraction and is not evidence for Gazebo behaviour.
