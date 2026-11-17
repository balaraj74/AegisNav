# Feature direction and scope decisions

## Product thesis

**AegisNav is a self-auditing, perception-aware rescue navigator.** It reasons over synthetic future hazard conditions, the reliability of its supplied sensing-confidence signal, and the feasibility of continuing or safely ending a mission. The label “self-aware” is used only in an engineering sense: the system monitors explicit confidence/state variables. It does not imply consciousness or human-like understanding.

## Candidate directions retained for future work

| Direction | Hypothesis / behavior | Current disposition |
|---|---|---|
| Perception-aware autonomy | Reduce trust in routes through regions where simulated visibility/localization quality degrades | **Core direction**; confidence is currently an external scalar, not estimated from raw perception |
| Counterfactual rescue twin | Compare routes under slow/nominal/rapid hazard-spread scenarios | **Stretch**; present model forecasts one configured scenario |
| Refuge-first autonomy | Select a designated reachable refuge when continuation or return is unacceptable | **Core direction** in the pure-Python mission supervisor |
| Time-bounded escape corridors | Reject a corridor expected to become unsafe before the robot clears it | Partly represented by time-indexed risk planning; explicit closure-time visualization remains future work |
| Active hazard intelligence | Move to an observation point to reduce uncertainty before entering a corridor | **Stretch**; no active sensing utility policy is implemented yet |
| Self-auditing decisions | Publish reason codes and human-readable explanations alongside route outputs | **Core direction**; ROS adapter emits a JSON decision trace |
| Scout/responder coordination | Share hazard information between two robots to improve rescue planning | **Out of deadline scope**; requires multi-robot coordination and fault-tested communication |

## Chosen competition slice

The minimum coherent competition demonstration should show:

1. A deterministic, time-indexed hazard field.
2. A risk-aware time-dependent route compared with a distance-only baseline under identical inputs.
3. A confidence signal that changes the mission policy.
4. A return-to-base feasibility assessment over the predicted hazard horizon.
5. Refuge/abort handling when a safe target or return plan is not established.
6. A structured decision trace and comparable metrics.

The source-only planning demo is implemented. ROS 2 integration and actuation remain unverified until tested on the target machine. The sample SDF world is geometric scaffolding, not photorealistic scenery.

## Deferred work order

1. Establish the ROS build and launch smoke test.
2. Connect the actual AMCL/localization source and complete Nav2 controller stack.
3. Verify path tracking, cancel/replan behavior and no-safe-route stop behavior at low speed.
4. Add the active sensing policy or multi-scenario forecast only after current behavior is tested.
5. Replace primitive scenery using an upstream disaster environment only after license and hardware compatibility review.
6. Run multiple scenarios/seeds and report distributions before making performance claims.

## Non-goals for this deadline

- Real-world deployment, emergency response certification or human safety claims.
- Physically valid fire/gas prediction.
- Calling the supplied confidence scalar a learned or calibrated perception-confidence estimate.
- Adding an LLM solely to make the project look more advanced.
- Implementing multiple robots, GPU-heavy digital twins, cloud services or custom high-fidelity physics before the algorithm works.
