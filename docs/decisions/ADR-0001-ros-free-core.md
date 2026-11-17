# ADR-0001: Keep the planning core ROS-independent

- **Status:** Accepted
- **Context:** Competition hardware and setup time are constrained; deterministic tests must work without bringing up Gazebo, DDS, RViz or a robot model.
- **Decision:** Domain objects, hazard forecasting, A*, mission policy and evaluation remain in the standard-library-only `aegisnav_core`. ROS adapters convert messages to/from those objects.
- **Consequences:** Fast unit tests and simpler debugging. ROS type conversion and runtime behavior require separate integration tests. The core cannot assume ROS clocks or simulator globals.
- **Rejected alternative:** Put all logic into a monolithic ROS node. That would make algorithm behavior difficult to test and encourage duplicated mission logic.
