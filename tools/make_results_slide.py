"""Render the batch results as a 1920x1080 slide PNG (HTML drawn by headless Chrome). Numbers come only from
artifacts/batch/summary.json (written by tools/analyze_batch.py); nothing is typed in by hand.

Usage: python3 tools/make_results_slide.py <out.png> [--batch-dir artifacts/batch]
"""
from __future__ import annotations

import argparse
import html
import json
import subprocess
import tempfile
from pathlib import Path

NAMES = {"sensing": "AegisNav (onboard sensing)", "baseline": "Shortest-path baseline"}
COLORS = {"sensing": "#2dd4bf", "baseline": "#ff6b5e"}


def pct(x: float) -> str:
    return f"{x:.0%}"


def build(summary: dict) -> str:
    modes = [m for m in ("sensing", "baseline") if m in summary and summary[m]["runs"] >= 5]
    if len(modes) < 2:
        raise SystemExit("need at least 5 runs of both sensing and baseline")
    seeds = sorted(set(summary["sensing"]["exposure_by_seed"]) & set(summary["baseline"]["exposure_by_seed"]), key=int)
    n = len(seeds)
    peak = max(max(summary[m]["exposure_by_seed"][s] for s in seeds) for m in modes) or 1.0
    cards = []
    for m in modes:
        d = summary[m]
        ex = [d["exposure_by_seed"][s] for s in seeds]
        mean = sum(ex) / len(ex)
        lo, hi = d["entered_hard_risk_ci"]
        dots = "".join(
            f'<i style="left:{100 * (i + 0.5) / len(ex):.1f}%;bottom:{100 * v / peak:.1f}%;background:{COLORS[m]}"></i>'
            for i, v in enumerate(ex))
        cards.append(f"""
        <div class="card" style="--c:{COLORS[m]}">
          <h2>{html.escape(NAMES[m])}</h2>
          <div class="big">{d['entered_hard_risk']}/{d['runs']}</div>
          <div class="lbl">runs that entered the deadly zone (risk ≥ 0.92)<br><span>95% CI {pct(lo)}–{pct(hi)}</span></div>
          <div class="big small">{mean:.1f}</div>
          <div class="lbl">mean hazard exposure, risk·s</div>
          <div class="plot"><b style="height:{100 * mean / peak:.1f}%;background:{COLORS[m]}33;border-top:3px solid {COLORS[m]}"></b>{dots}</div>
          <div class="lbl">each dot = one seed</div>
          <div class="lbl">{d['complete']}/{d['runs']} completed all 5 points and returned to base</div>
        </div>""")
    paired = summary.get("paired_exposure", {}).get("sensing-baseline")
    pair_txt = ""
    if paired:
        lo, hi = paired["ci"]
        pair_txt = (f'<div class="pair">Same {len(paired["seeds"])} seeds, same fires: exposure difference '
                    f'<b>{paired["mean"]:+.1f} risk·s</b> (95% bootstrap CI {lo:+.1f} to {hi:+.1f}), AegisNav minus baseline</div>')
    return f"""<!doctype html><html><head><meta charset="utf-8"><style>
    html,body{{margin:0;width:1920px;height:1080px;background:#0b0f14;color:#e8eef5;font-family:'DejaVu Sans',sans-serif}}
    .wrap{{padding:60px 90px}} h1{{font-size:54px;margin:0 0 6px}} .sub{{font-size:26px;color:#9fb0c3;margin-bottom:34px}}
    .row{{display:flex;gap:40px}} .card{{flex:1;background:#121923;border:1px solid #243041;border-radius:18px;padding:26px 34px;border-top:6px solid var(--c)}}
    .card h2{{margin:0 0 6px;font-size:30px;color:var(--c)}} .big{{font-size:104px;font-weight:700;line-height:1.05}} .big.small{{font-size:60px;margin-top:12px}}
    .lbl{{font-size:22px;color:#9fb0c3}} .lbl span{{color:#7d8fa3}}
    .plot{{position:relative;height:150px;margin:8px 0 4px;border-bottom:2px solid #31405a}} .plot b{{position:absolute;left:0;right:0;bottom:0;display:block}}
    .plot i{{position:absolute;width:16px;height:16px;border-radius:50%;margin-left:-8px;margin-bottom:-8px}}
    .pair{{margin-top:28px;font-size:28px;color:#e8eef5}} .note{{position:absolute;left:90px;bottom:40px;right:90px;font-size:21px;color:#7d8fa3}}
    </style></head><body><div class="wrap">
    <h1>Measured over repeated full missions</h1>
    <div class="sub">Gazebo + Nav2, {n} matched seeds, 3 seeded random fires each. Exposure measured against the real hazard field.</div>
    <div class="row">{''.join(cards)}</div>{pair_txt}
    </div><div class="note">Simulation results. The hazard model, the sensor and the confidence estimate are synthetic. {n} runs per mode gives wide intervals; no percentage improvement is claimed.</div></body></html>"""


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--batch-dir", default="artifacts/batch")
    args = ap.parse_args()
    summary = json.loads((Path(args.batch_dir) / "summary.json").read_text())
    page = build(summary)
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "slide.html"
        f.write_text(page, encoding="utf-8")
        subprocess.run(["google-chrome", "--headless=new", "--disable-gpu", "--no-sandbox", "--hide-scrollbars",
                        f"--screenshot={Path(args.out).resolve()}", "--window-size=1920,1080", f.as_uri()],
                       check=True, capture_output=True, timeout=60)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
