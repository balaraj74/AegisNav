# Rule 40 — Dependencies, hardware and licensing

- Core logic stays standard-library-only and runnable without ROS/Gazebo.
- Before adding a package, explain why it is needed, its version compatibility, installation burden, license, RAM/VRAM impact and fallback.
- Target laptop profile is a mobile workstation with 16 GB RAM and an RTX 2050 4 GB GPU. Prefer low-poly geometry and moderate camera/render settings for interactive simulation.
- Do not bundle external scenes, meshes, textures, datasets or robot assets without a license/redistribution review.
- No runtime cloud requests or telemetry by default.
- Avoid Isaac Sim as a required dependency for this target machine; use Gazebo Harmonic for algorithm verification and optional external assets for improved visuals.
