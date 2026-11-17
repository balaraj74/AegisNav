"""Seeded baseline-vs-AegisNav experiment runner producing JSON and an SVG summary."""
from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter
from html import escape
from pathlib import Path

from .models import Cell, RoutePlan
from .simulation import Mode, TrialResult, TruthField, run_trial, seeded_dynamic_scenario
from .svg_report import render_map_svg
from .timeline_report import render_timeline_svg

METRICS = (
    "hazard_exposure", "time_near_hazard_s", "time_in_hazard_s", "replans",
    "path_length_m", "mission_time_s", "planning_time_ms_mean",
)
DEFINITIONS = {
    "hazard_exposure": "sum over 1 s ticks of ground-truth risk [0,1] at the robot's cell (risk-seconds)",
    "time_near_hazard_s": "ticks with ground-truth risk >= near_risk (0.5)",
    "time_in_hazard_s": "ticks with ground-truth risk >= hard_risk_threshold (0.92)",
    "replans": "replacement goals dispatched after cancelling an active goal",
    "path_length_m": "distance advanced by the simulated robot, metres",
    "mission_time_s": "simulated seconds until a terminal outcome or timeout",
    "planning_time_ms_mean": "wall-clock mean per decision cycle on the host; not deterministic",
    "mission_completion_rate": "fraction of trials in which the robot reached the target cell",
    "safe_return_rate": "fraction of trials that reached the target, returned to base, and never had ground-truth risk >= 0.92",
    "safe_outcome_rate": "fraction of trials that ended at base or a refuge (not timeout/plan failure) with ground-truth risk never >= 0.92; includes early returns, so it does not measure mission success",
    "abort_rate": "fraction of trials ending in ABORTED or PLAN_FAILED",
}


def _stats(values: list[float]) -> dict:
    return {
        "mean": round(statistics.fmean(values), 4),
        "stdev": round(statistics.stdev(values), 4) if len(values) > 1 else 0.0,
        "min": round(min(values), 4), "max": round(max(values), 4),
    }


def summarize(results: list[TrialResult]) -> dict:
    n = len(results)
    out = {
        "trials": n,
        "outcomes": dict(Counter(r.outcome.value for r in results)),
        "mission_completion_rate": round(sum(r.reached_target for r in results) / n, 4),
        "safe_return_rate": round(sum(r.safe_return for r in results) / n, 4),
        "safe_outcome_rate": round(sum(r.safe_outcome for r in results) / n, 4),
        "abort_rate": round(sum(r.outcome.value in ("aborted", "plan_failed") for r in results) / n, 4),
    }
    for name in METRICS:
        out[name] = _stats([float(getattr(r, name)) for r in results])
    out["failed_seeds"] = sorted(r.seed for r in results if not r.reached_target)
    return out


def run_experiment(seeds: list[int]) -> tuple[dict, dict[str, list[TrialResult]]]:
    per_mode: dict[str, list[TrialResult]] = {m.value: [] for m in Mode}
    for seed in seeds:
        ds = seeded_dynamic_scenario(seed)
        truth = TruthField(ds)  # one truth per seed, shared by both modes
        for mode in Mode:
            per_mode[mode.value].append(run_trial(ds, truth, mode))
    return {
        "scenario_family": "corridor_hazard_spread_v2 + seeded ignition/confidence events",
        "seeds": seeds,
        "metric_definitions": DEFINITIONS,
        "model_notice": "Synthetic hazard and kinematic robot abstraction; no Gazebo/Nav2 involved. Not evidence of real-world safety.",
        "summary": {mode: summarize(res) for mode, res in per_mode.items()},
        "trials": {mode: [r.metrics_dict() for r in res] for mode, res in per_mode.items()},
    }, per_mode


def render_summary_svg(path: Path, report: dict) -> None:
    """Per-trial dots plus mean bar for the two headline exposure metrics."""
    panels = [("hazard_exposure", "Ground-truth hazard exposure (risk·s)"), ("time_in_hazard_s", "Time at risk ≥ 0.92 (s)")]
    modes = [m.value for m in Mode]
    palette = ("#2b6cb0", "#2f855a", "#c05621")
    colors = {m: palette[i % len(palette)] for i, m in enumerate(modes)}
    labels = {Mode.AEGISNAV.value: "omniscient", Mode.AEGISNAV_SENSING.value: "sensing", Mode.BASELINE.value: "baseline"}
    spacing = 240 // max(1, len(modes) - 1)  # columns across a 300 px panel
    w, h, pad = 760, 340, 50
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" font-family="sans-serif" font-size="12">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="{pad}" y="22" font-size="14" font-weight="bold">AegisNav (omniscient, onboard sensing) vs baseline — {report["summary"][modes[0]]["trials"]} seeded trials per mode (synthetic model)</text>']
    for p, (key, label) in enumerate(panels):
        x0 = pad + p * 360
        top, bottom = 60, h - 50
        peak = max(max(t[key] for t in report["trials"][m]) for m in modes) or 1.0
        parts.append(f'<text x="{x0}" y="{top - 14}">{escape(label)}</text>')
        parts.append(f'<line x1="{x0}" y1="{bottom}" x2="{x0 + 300}" y2="{bottom}" stroke="#444"/>')
        parts.append(f'<text x="{x0 - 6}" y="{top + 4}" text-anchor="end">{peak:.1f}</text><text x="{x0 - 6}" y="{bottom + 4}" text-anchor="end">0</text>')
        for i, m in enumerate(modes):
            cx = x0 + 30 + i * spacing
            mean = report["summary"][m][key]["mean"]
            y_mean = bottom - (bottom - top) * mean / peak
            parts.append(f'<rect x="{cx - 30}" y="{y_mean:.1f}" width="60" height="{bottom - y_mean:.1f}" fill="{colors[m]}" opacity="0.25"/>')
            trials = report["trials"][m]
            for j, t in enumerate(trials):
                y = bottom - (bottom - top) * t[key] / peak
                dx = (j % 7 - 3) * 8
                parts.append(f'<circle cx="{cx + dx}" cy="{y:.1f}" r="3" fill="{colors[m]}"/>')
            parts.append(f'<text x="{cx}" y="{bottom + 16}" text-anchor="middle">{escape(labels.get(m, m))}</text>')
            parts.append(f'<text x="{cx}" y="{bottom + 30}" text-anchor="middle" fill="#333">mean {mean:.2f}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def _executed_route(result: TrialResult) -> RoutePlan:
    cells = [c for i, c in enumerate(result.trajectory) if i == 0 or c != result.trajectory[i - 1]]
    return RoutePlan(tuple(cells), True, "executed", distance_m=result.path_length_m)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run seeded baseline vs AegisNav closed-loop trials.")
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--first-seed", type=int, default=0)
    parser.add_argument("--showcase-seed", type=int, default=None, help="Also write trace + map for this seed")
    parser.add_argument("--output-dir", default="artifacts/eval")
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials must be at least 1")
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    seeds = list(range(args.first_seed, args.first_seed + args.trials))
    report, per_mode = run_experiment(seeds)
    (out / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    render_summary_svg(out / "summary.svg", report)
    if args.showcase_seed is not None:
        ds = seeded_dynamic_scenario(args.showcase_seed)
        truth = TruthField(ds)
        runs = {m: run_trial(ds, truth, m) for m in Mode}
        (out / f"showcase_seed{args.showcase_seed}_trace.json").write_text(json.dumps(
            {m.value: {"metrics": r.metrics_dict(), "trace": r.trace} for m, r in runs.items()}, indent=2) + "\n", encoding="utf-8")
        for mode, run in runs.items():
            render_timeline_svg(out / f"showcase_seed{args.showcase_seed}_{mode.value}_timeline.svg", run.trace,
                                f"{mode.value} — seed {args.showcase_seed} (simulated)")
        final_frame = truth.frame(min(len(truth.frames) - 1, max(r.mission_time_s for r in runs.values())))
        render_map_svg(out / f"showcase_seed{args.showcase_seed}_map.svg", ds.base.grid, final_frame,
                       ds.base.start, ds.base.target, _executed_route(runs[Mode.AEGISNAV]),
                       other_route=_executed_route(runs[Mode.BASELINE]),
                       title=f"Executed routes, seed {args.showcase_seed} (truth hazard at end of longer run)")
    print(json.dumps({m: s for m, s in report["summary"].items()}, indent=2))


if __name__ == "__main__":
    main()
