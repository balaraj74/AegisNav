"""Run deterministic local demos without requiring ROS or Gazebo."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .evaluation import TrialMetrics, write_metrics
from .forecast import HazardForecaster
from .models import PlannerConfig
from .planning import RiskAwareAStar
from .scenarios import corridor_scenario
from .svg_report import render_map_svg


def run_demo(output_dir: Path, *, risk_weight: float = 12.0) -> dict:
    scenario = corridor_scenario()
    forecast = HazardForecaster().forecast(
        scenario.grid, scenario.initial_risk, scenario.initial_uncertainty,
        scenario.sources, scenario.forecast_config,
    )
    planner = RiskAwareAStar()
    risk_plan = planner.plan(
        scenario.grid, forecast.frames, scenario.start, scenario.target,
        PlannerConfig(hazard_weight=risk_weight, uncertainty_weight=2.5, hard_risk_threshold=0.92),
        time_step_s=scenario.forecast_config.time_step_s,
    )
    baseline_config = PlannerConfig(hazard_weight=0.0, uncertainty_weight=0.0, hard_risk_threshold=0.99)
    baseline_plan = planner.plan(
        scenario.grid, forecast.frames, scenario.start, scenario.target,
        baseline_config,
        time_step_s=scenario.forecast_config.time_step_s,
    )
    baseline_return = None
    if baseline_plan.success:
        baseline_return = planner.plan(
            scenario.grid, forecast.frames, scenario.target, scenario.base,
            baseline_config, start_time_step=baseline_plan.duration_steps,
            time_step_s=scenario.forecast_config.time_step_s,
        )
    return_plan = None
    if risk_plan.success:
        return_plan = planner.plan(
            scenario.grid, forecast.frames, scenario.target, scenario.base,
            PlannerConfig(hazard_weight=risk_weight, uncertainty_weight=2.5, hard_risk_threshold=0.92),
            start_time_step=risk_plan.duration_steps,
            time_step_s=scenario.forecast_config.time_step_s,
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    render_map_svg(
        output_dir / "aegisnav_route_comparison.svg", scenario.grid,
        forecast.at_step(min(12, len(forecast.frames) - 1)), scenario.start, scenario.target,
        risk_plan, other_route=baseline_plan,
        title="AegisNav — predictive risk-aware route comparison",
    )
    common_risk_threshold = 0.92
    exposure_budget = 18.0
    baseline_safe_return = bool(
        baseline_plan.success and baseline_return and baseline_return.success
        and baseline_plan.max_risk_encountered < common_risk_threshold
        and baseline_return.max_risk_encountered < common_risk_threshold
        and baseline_plan.hazard_exposure <= exposure_budget
        and baseline_return.hazard_exposure <= exposure_budget
    )
    risk_aware_safe_return = bool(
        risk_plan.success and return_plan and return_plan.success
        and risk_plan.max_risk_encountered < common_risk_threshold
        and return_plan.max_risk_encountered < common_risk_threshold
        and risk_plan.hazard_exposure <= exposure_budget
        and return_plan.hazard_exposure <= exposure_budget
    )
    metrics = [
        TrialMetrics.from_plan(
            scenario.scenario_id, "distance_only_baseline", baseline_plan,
            safe_return_success=baseline_safe_return, risk_threshold=common_risk_threshold,
            notes="One-shot distance-only A* on the same map and forecast; no replanning loop is measured. Hazard/uncertainty penalties are disabled. Route-level critical-risk flag uses the shared 0.92 evaluation threshold; safe-return success checks both outbound and return paths against that threshold and the 18.0 exposure budget.",
        ),
        TrialMetrics.from_plan(
            scenario.scenario_id, "aegisnav_risk_aware", risk_plan,
            safe_return_success=risk_aware_safe_return, risk_threshold=common_risk_threshold,
            notes="One-shot time-indexed risk-aware A*; same scenario and forecast as baseline; dynamic replan count is not measured; synthetic model only.",
        ),
    ]
    write_metrics(output_dir / "metrics.json", metrics)
    summary = {
        "scenario_id": scenario.scenario_id,
        "forecast_frames": len(forecast.frames),
        "risk_aware": metrics[1].to_dict(),
        "baseline": metrics[0].to_dict(),
        "risk_aware_return_path_found": bool(return_plan and return_plan.success),
        "baseline_return_path_found": bool(baseline_return and baseline_return.success),
        "risk_aware_safe_return_success": risk_aware_safe_return,
        "baseline_safe_return_success": baseline_safe_return,
        "artifacts": [str(output_dir / "aegisnav_route_comparison.svg"), str(output_dir / "metrics.json")],
        "model_notice": "Synthetic hazard model for repeatable simulation only; not a real fire/gas physics model.",
    }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Run AegisNav's deterministic hazard-aware planning demo.")
    parser.add_argument("--output-dir", default="artifacts/demo", help="Directory for SVG and JSON artifacts")
    parser.add_argument("--risk-weight", type=float, default=12.0, help="Hazard exposure weight for the route objective")
    args = parser.parse_args()
    if args.risk_weight < 0:
        parser.error("--risk-weight must be non-negative")
    summary = run_demo(Path(args.output_dir), risk_weight=args.risk_weight)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
