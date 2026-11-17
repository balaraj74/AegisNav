# Source review

Accessed 2026-10-29 from the development host (Ubuntu 26.04.1, no ROS/Gazebo installed). Pages were fetched with `curl` and read as text; the WebFetch summariser tool was unavailable in this session. Only the sources listed below were inspected. The other URLs in the directive (Nav2 planner/controller configuration, plugin tutorial, Gazebo install pages, ROS install page, `ros_gz` repository) were **not** read in this session and nothing here depends on them.

## 1. Nav2 `FollowPath` action (Jazzy API page)
- URL: https://api.nav2.org/actions/jazzy/followpath.html
- Inspected: whole page text.
- Verified: goal fields `path` (`nav_msgs/Path`), `controller_id`, `goal_checker_id`, `progress_checker_id` (strings). Result carries `error_code` (uint16: NONE, UNKNOWN, INVALID_CONTROLLER, TF_ERROR, INVALID_PATH, PATIENCE_EXCEEDED, FAILED_TO_MAKE_PROGRESS, NO_VALID_CONTROL, CONTROLLER_TIMED_OUT) and `error_msg`.
- Decision: `planner_node` fills exactly those goal fields, and any terminal status other than SUCCEEDED/CANCELED is treated as `GoalOutcome.FAILED` (feeds the failure latch in `ExecutionController`).
- Unresolved: the *values* of controller/goal-checker/progress-checker IDs must match the controller server's parameter file. The defaults in `planner_node.py` (`FollowPath`, `general_goal_checker`, `progress_checker`) were not checked against a running Nav2 and `execute_with_follow_path` therefore stays `false`. The action server name `/follow_path` is likewise unconfirmed.

## 2. Gazebo Harmonic — ROS installation / compatibility
- URL: https://gazebosim.org/docs/harmonic/ros_installation/
- Inspected: compatibility section and table.
- Verified: ROS 2 Jazzy + Gazebo Harmonic is the recommended pair on Ubuntu 24.04. The page's current recommendation for new users is Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty; Jazzy with Jetty is marked incompatible.
- Consequence for this project: the development host runs Ubuntu 26.04, which is **not** a Jazzy platform. A live AegisNav simulation needs an Ubuntu 24.04 machine/VM/container (the competition target in `AGENTS.md`). No global system change was made here.

## 3. `juanscelyg/disaster_world`
- URL: https://github.com/juanscelyg/disaster_world (README fetched via raw.githubusercontent.com).
- Verified: ROS 2 Jazzy + Gazebo Harmonic; two large outdoor worlds (Baylands Park, Nathan Benderson Park) built from Gazebo Fuel models; needs internet on first launch to download Fuel models (cached in `~/.gz/fuel`); README describes the worlds as fictional scenarios. The GitHub license API returned 404 (no license detected by GitHub); the repository files themselves were not inspected for a license, and Fuel model licenses are per-model and were not reviewed.
- Decision: **not adopted.** The worlds are large outdoor parks with no relation to AegisNav's 24×16 indoor occupancy grid, the license/redistribution status is unconfirmed, and runtime downloads conflict with the "no network in the runtime core" rule. The bundled lightweight SDF world remains the only world. Any future adoption requires a geometry alignment check, license review recorded in `THIRD_PARTY_NOTICES.md`, and a GPU measurement on the RTX 2050.

## Novelty
No literature search was performed. No novelty claim is made; the contribution is an engineering study of one decision policy on a synthetic hazard model.

## 4. Installed packages inspected in the `ros:jazzy` image (2026-10-29)
- `nav2_regulated_pure_pursuit_controller`: plugin class `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` (plugin XML) and parameter names (from the shared library's strings) used in `nav2_params.yaml`.
- `nav2_map_server`: parameters `yaml_filename`, `topic_name`, `frame_id` (library strings) — used to publish the planning grid on `/aegisnav/planning_map`.
- Runtime confirmation: `ros2 action list -t` showed `/follow_path [nav2_msgs/action/FollowPath]`; controller created `FollowPath` of the RPP type; lifecycle states `active`.
