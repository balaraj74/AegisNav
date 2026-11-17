# Research positioning, novelty discipline and evaluation plan

## Candidate research question

**Can planning over time-indexed hazard forecasts, while explicitly accounting for uncertainty and evaluating target-to-base feasibility, reduce predicted hazard exposure and mission failures compared with distance-only planning in controlled dynamic-hazard scenarios?**

This is a hypothesis to test, not an established result.

## Proposed contribution to evaluate

The defensible contribution is the particular combination and implementation of:

1. time-indexed route costs on a grid with hard critical-risk exclusion;
2. a mission feasibility check that evaluates the return route at predicted target arrival time;
3. a confidence-sensitive decision policy with explicit continue/reassess/return/refuge/abort outputs;
4. a reproducible evaluation harness with shared scenarios, reason-code traces and comparable baselines.

Each item has prior art in robotics and planning. The mere presence of hazard prediction, risk-aware cost, confidence checks, A*, Nav2 or explainable decision messages is not claimed as novel. Search current papers and verify overlap before describing the work as novel in a report, submission or paper.

## Baselines

Minimum baselines should use the same map, start, target, source locations, forecast assumptions and hardware/time budget:

- **B0: Distance-only A*** — hazard and uncertainty weights disabled; same static obstacles.
- **B1: Reactive hazard-cost A*** — route against the current hazard layer only, without forecasting future layers.
- **AegisNav: Time-indexed risk-aware A*** — forecast risk and uncertainty, hard threshold, plus mission-level return feasibility and confidence policy.

B1 requires a separate implementation/adapter before it can be fairly reported. Do not imply it is included in the current demo.

## Metrics

Report at least:

- mission completion and safe-return success;
- integrated hazard exposure over the planned/observed route;
- time spent above a defined risk level;
- distance and estimated time;
- number of replans and reason codes;
- hard threshold violations;
- refuge success when return is infeasible;
- planner runtime and expansion count.

For the final comparison, run many seeds/scenarios, report the mean and dispersion or confidence interval, list failed trials, and use identical scenario conditions. The current CLI is a single deterministic illustration, not a statistical benchmark.

## Threats to validity

- The hazard diffusion model is synthetic and non-physical.
- The initial confidence signal is externally supplied; it is not estimated from image/LiDAR perception.
- The grid and source configuration are known to the scenario designer.
- Route safety is calculated from prediction layers, which may be wrong.
- Estimated time uses a constant nominal speed and omits acceleration, dynamic obstacles, controller tracking error and communication delays.
- A candidate route is not necessarily executable by a real Nav2 controller or safe under local obstacle avoidance.
- Only the 24×16 sample scenario is bundled in the core demo.

## Language policy

Allowed: “synthetic forecast”, “predicted risk under our configured model”, “simulation result”, “candidate contribution”, “observed in this scenario”.

Not allowed without separate evidence: “physically accurate fire prediction”, “guaranteed safe”, “real-world deployable”, “conscious”, “AI understands the scene”, “novel first-of-its-kind”, or unmeasured improvement percentages.
