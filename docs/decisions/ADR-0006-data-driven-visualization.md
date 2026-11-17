# ADR-0006: Hazard visuals are rendered from the forecast topic, never the reverse

- **Status:** Accepted (verified in headless Gazebo + RViz under Xvfb, 2026-10-29)
- **Context:** The world contained static "fire/smoke" models at the two configured sources. They did not follow the dynamic ignition, duplicated configuration, and their glow cylinders (z 0.03–0.29 m) crossed the lidar plane (0.28 m), creating phantom obstacles in the lower gate and in open floor.
- **Decision:**
  1. Remove all static hazard models. `aegisnav_visualization` renders forecast **layer 0** (the current estimate) as flat, emissive, collision-free floor tiles in Gazebo — one model per display band (risk ≥ 0.25 / 0.5 / 0.75), replaced via the bridged `/world/<world>/create` and `/remove` services every `tile_period_s` (2 s). Calls have a 3 s deadline; models whose reply was lost are removed later (observed: per-tile spawning lost replies and stalled the display).
  2. Hazard visuals carry visibility flag `0x2`; the lidar's `<visibility_mask>` clears that bit. Verified: a 1 m tall flagged box 0.53 m ahead was invisible (range 3.33 m), the same box unflagged was seen (0.43 m).
  3. RViz shows the **current** estimate (warm, floor level) and one **predicted** layer (+10 s, violet, raised) on separate topics, plus base/refuges/goal and a decision text overlay labelled "(simulated)".
  4. Scene polish uses primitives only (no third-party assets): concrete materials, two shadowless fill lights, generated pads and doorway threshold bars below 1 cm, interior walls 0.7 m / perimeter 1.0 m (above the lidar plane), rubble only outside the perimeter where the 2D lidar cannot see it.
- **Consequences:** What the viewer sees as hazard is exactly the data the planner consumes (current layer), at display thresholds that are not planning thresholds. Gazebo display lags the forecast by up to one refresh period. GPU frame rate on the RTX 2050 has not been measured; all renders so far are software (llvmpipe).
