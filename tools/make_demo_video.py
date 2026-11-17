"""Cut the final demo video from two recorded takes (same seed): AegisNav with onboard sensing, and the baseline.

Captions and timing come only from the takes' logs and recorder timelines; the results slide comes only from
artifacts/batch/summary.json (tools/make_results_slide.py). Gazebo frames get a light colour grade (bloom, contrast,
vignette, grain); it is a grade of the real render, not extra realism.

Usage: python3 tools/make_demo_video.py --main <take_dir> --base <take_dir> --slide <results.png> --out <video.mp4> [--plan]
Take dir: gazebo.mkv, console.mkv, sim.log, metrics.json (with "timeline"), capture_start.txt  (see tools/record_take.sh)
"""
from __future__ import annotations

import argparse
import bisect
import json
import re
import subprocess
from pathlib import Path

BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
GRADE = ("crop=1600:868:0:32,format=gbrp,split=2[a][b];[b]gblur=sigma=16,eq=brightness=-0.08:contrast=1.5[bl];"
         "[a][bl]blend=all_mode=screen:all_opacity=0.30,format=yuv420p,eq=contrast=1.08:saturation=1.12:gamma=0.97,"
         "unsharp=5:5:0.5,vignette=PI/6,noise=alls=2:allf=t")
STAMP = re.compile(r"\[(\d+\.\d+)\] \[[a-z_.]+\]: (.*)")
BG = "0x0b0f14"


class Take:
    def __init__(self, d: Path) -> None:
        self.dir = d
        self.start = float((d / "capture_start.txt").read_text())
        self.dur = float(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0",
                                                  str(d / "gazebo.mkv")]))
        self.events = [(float(m.group(1)) - self.start, m.group(2)) for line in (d / "sim.log").read_text(errors="ignore").splitlines()
                       if (m := STAMP.search(line))]
        self.meta = json.loads((d / "metrics.json").read_text())
        tl = self.meta.get("timeline") or []
        self.timeline = [(w - self.start, ms, ex) for w, ms, ex, *_ in tl]  # (video_s, mission_s, exposure)

    def video_at_mission(self, mission_s: float) -> float:
        ms = [t[1] for t in self.timeline]
        i = max(1, min(len(ms) - 1, bisect.bisect_left(ms, mission_s)))
        (v0, m0, _), (v1, m1, _) = self.timeline[i - 1], self.timeline[i]
        return v0 if m1 == m0 else v0 + (v1 - v0) * (mission_s - m0) / (m1 - m0)

    def exposure_at(self, mission_s: float) -> float:
        ms = [t[1] for t in self.timeline]
        i = max(1, min(len(ms) - 1, bisect.bisect_left(ms, mission_s)))
        (_, m0, e0), (_, m1, e1) = self.timeline[i - 1], self.timeline[i]
        return e0 if m1 == m0 else e0 + (e1 - e0) * (mission_s - m0) / (m1 - m0)


def esc(path: Path) -> str:
    return str(path)


def run(cmd: list[str]) -> None:
    subprocess.run(cmd, check=True, capture_output=True)


def card(work: Path, name: str, lines: list[tuple[str, int, str, int, str]], seconds: float) -> Path:
    filt = []
    for i, (text, size, font, y, color) in enumerate(lines):
        f = work / f"{name}_{i}.txt"
        f.write_text(text)
        filt.append(f"drawtext=fontfile={font}:textfile={f}:fontsize={size}:fontcolor={color}:x=(w-text_w)/2:y={y}")
    out = work / f"{name}.mp4"
    run(["ffmpeg", "-y", "-loglevel", "error", "-f", "lavfi", "-i", f"color=c={BG}:s=1920x1080:r=30:d={seconds}",
         "-f", "lavfi", "-i", "anullsrc=r=44100:cl=stereo", "-shortest", "-vf", ",".join(filt) + ",fade=t=in:d=0.6,format=yuv420p",
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-c:a", "aac", str(out)])
    return out


def still(work: Path, name: str, png: Path, seconds: float) -> Path:
    out = work / f"{name}.mp4"
    run(["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-framerate", "30", "-t", str(seconds), "-i", str(png),
         "-f", "lavfi", "-i", "anullsrc=r=44100:cl=stereo", "-shortest", "-vf", "scale=1920:1080,fade=t=in:d=0.5,format=yuv420p",
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-c:a", "aac", str(out)])
    return out


def header(subtitle: str) -> str:
    return (f"drawtext=fontfile={BOLD}:text='AegisNav':fontsize=44:fontcolor=white:x=40:y=38,"
            f"drawtext=fontfile={REG}:text='{subtitle}':fontsize=26:fontcolor=0x9fb0c3:x=300:y=52")


def clip(work: Path, i: int, take_main: Take, start: float, end: float, speed: float, caption: str, *,
         take_other: Take | None = None, other_start: float = 0.0, labels: tuple[str, str] = ("", "")) -> Path:
    out = work / f"seg{i:03d}.mp4"
    cap, spd = work / f"cap{i}.txt", work / f"spd{i}.txt"
    cap.write_text(caption)
    spd.write_text(f"Speed x{speed:g}")
    dur = end - start
    if take_other is None:  # Gazebo (graded) + operator console
        inputs = ["-ss", f"{start:.2f}", "-t", f"{dur:.2f}", "-i", str(take_main.dir / "gazebo.mkv"),
                  "-ss", f"{start:.2f}", "-t", f"{dur:.2f}", "-i", str(take_main.dir / "console.mkv")]
        panes = (f"[0:v]{GRADE},scale=1032:560[g];[1:v]scale=796:560[c];[g][c]hstack=2[row];color=c={BG}:s=1920x1080:r=30[bg];"
                 f"[bg][row]overlay=44:190:shortest=1,"
                 f"drawtext=fontfile={REG}:text='3D view (the real hazards)':fontsize=22:fontcolor=0x9fb0c3:x=44:y=152,"
                 f"drawtext=fontfile={REG}:text='Operator console (what the robot knows)':fontsize=22:fontcolor=0x9fb0c3:x=1100:y=152")
        sub = "Hazard-aware rescue navigation  |  ROS 2 + Gazebo + Nav2 simulation"
    else:  # two graded Gazebo views, same seed
        inputs = ["-ss", f"{start:.2f}", "-t", f"{dur:.2f}", "-i", str(take_main.dir / "gazebo.mkv"),
                  "-ss", f"{other_start:.2f}", "-t", f"{dur:.2f}", "-i", str(take_other.dir / "gazebo.mkv")]
        panes = (f"[0:v]{GRADE},scale=920:499[l];[1:v]{GRADE},scale=920:499[r];[l][r]hstack=2[row];color=c={BG}:s=1920x1080:r=30[bg];"
                 f"[bg][row]overlay=20:200:shortest=1,"
                 f"drawtext=fontfile={BOLD}:text='{labels[0]}':fontsize=28:fontcolor=0x2dd4bf:x=20:y=154,"
                 f"drawtext=fontfile={BOLD}:text='{labels[1]}':fontsize=28:fontcolor=0xff6b5e:x=980:y=154")
        sub = "Same seed, same fires, same mission time"
    vf = (f"{panes},{header(sub)},drawtext=fontfile={BOLD}:textfile={spd}:fontsize=26:fontcolor=0xffc857:x=w-text_w-40:y=52,"
          f"drawtext=fontfile={BOLD}:textfile={cap}:fontsize=36:fontcolor=white:x=(w-text_w)/2:y=900,"
          f"setpts=PTS/{speed},fps=30,format=yuv420p")
    run(["ffmpeg", "-y", "-loglevel", "error", *inputs, "-f", "lavfi", "-i", "anullsrc=r=44100:cl=stereo", "-shortest",
         "-filter_complex", vf + "[v]", "-map", "[v]", "-map", "2:a", "-c:v", "libx264", "-preset", "veryfast", "-crf", "19",
         "-c:a", "aac", str(out)])
    return out


def part_a_segments(t: Take, target_body_s: float) -> list[tuple[float, float, float, str]]:
    key, goals = [], []
    hazards = victims = 0
    base_t = None
    for ts, text in t.events:
        if "Hazard discovered" in text:
            hazards += 1
            key.append((ts, f"The sensor discovers hazard {hazards}. The robot did not know about it before seeing it."))
        elif text.startswith("Cancelling FollowPath"):
            key.append((ts, "The planner cancels the route and replaces it with a safer one."))
        elif "Victim detected" in text:
            victims += 1
            key.append((ts, f"Victim {victims} found by the onboard sensor (line of sight, within 4 m)."))
        elif "Triage" in text and (m := re.search(r"now (base|inspection point (\d))", text)):
            label = "base" if m.group(1) == "base" else f"inspection point {m.group(2)}"
            key.append((ts, "All points inspected. Returning to base." if label == "base" else f"Next: {label}, the lowest-exposure safe route."))
            goals.append((ts, label))
    first = next(ts for ts, text in t.events if text.startswith("Decision"))
    end = min(t.dur - 0.5, t.dur - 0.5 if t.meta.get("end_reason") != "base" else t.dur - 0.5)
    windows: list[list] = []
    for ts, cap in key:
        a, b = max(first, ts - 3.0), min(end, ts + 8.0)
        if windows and a <= windows[-1][1]:
            windows[-1][1] = max(windows[-1][1], b)
            windows[-1][2].append((ts, cap))
        else:
            windows.append([a, b, [(ts, cap)]])
    slow_total = sum(b - a for a, b, _ in windows)
    fast_total = (end - first) - slow_total
    speed = max(4.0, round(fast_total / max(10.0, target_body_s - slow_total / 2.0)))

    def goal_at(ts):
        label = "inspection point 1"
        for gt, name in goals:
            if gt <= ts:
                label = name
        return label

    segs, cur = [], first
    for a, b, caps in windows:
        if a - cur > 1.0:
            segs.append((cur, a, speed, f"Heading to {goal_at(cur)}."))
        for i, (ts, cap) in enumerate(caps):
            sa = a if i == 0 else ts
            sb = caps[i + 1][0] if i + 1 < len(caps) else b
            if sb - sa > 0.3:
                segs.append((sa, sb, 2.0, cap))
        cur = b
    if end - cur > 1.0:
        segs.append((cur, end, speed, f"Heading to {goal_at(cur)}."))
    return segs


def comparison_windows(main: Take, base: Take, count: int = 2, span_s: float = 18.0) -> list[tuple[float, float, float, float]]:
    """Mission-time windows (start, end, baseline exposure gain, aegisnav exposure gain) where the baseline gains most."""
    cands = []
    m_end = base.timeline[-1][1]
    m = 5.0
    while m + span_s < m_end:
        gain_b = base.exposure_at(m + span_s) - base.exposure_at(m)
        cands.append((gain_b, m))
        m += 2.0
    chosen = []
    for gain_b, m in sorted(cands, reverse=True):
        if gain_b < 0.5 or any(abs(m - c) < span_s + 10 for _, c in chosen):
            continue
        chosen.append((gain_b, m))
        if len(chosen) == count:
            break
    return sorted((m, m + span_s, g, main.exposure_at(m + span_s) - main.exposure_at(m)) for g, m in chosen)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--main", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--slide", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--plan", action="store_true")
    ap.add_argument("--body-s", type=float, default=70.0)
    args = ap.parse_args()
    main_t, base_t = Take(Path(args.main)), Take(Path(args.base))
    work = Path(args.main) / "edit_final"
    work.mkdir(exist_ok=True)
    segs = part_a_segments(main_t, args.body_s)
    cmp = comparison_windows(main_t, base_t)
    for s in segs:
        print(f"A {s[0]:7.1f}-{s[1]:7.1f} x{s[2]:g} {s[3]}")
    for m0, m1, gb, ga in cmp:
        print(f"B mission {m0:.0f}-{m1:.0f} s: baseline exposure +{gb:.1f}, AegisNav +{ga:.1f}")
    if args.plan:
        return
    parts = [card(work, "title", [("AegisNav", 110, BOLD, 330, "white"),
                                   ("Hazard-aware rescue navigation in simulation", 46, REG, 470, "white"),
                                   ("The robot does not know where the hazards are. It finds them with a simulated", 32, REG, 590, "0x9fb0c3"),
                                   ("onboard sensor, replans around them, and decides where to go next.", 32, REG, 636, "0x9fb0c3"),
                                   ("Time is sped up; the speed is shown on screen.", 28, REG, 760, "0xffc857")], 6)]
    parts += [clip(work, i, main_t, a, b, sp, cap) for i, (a, b, sp, cap) in enumerate(segs)]
    parts.append(card(work, "mid", [("Same fires. Same seed.", 80, BOLD, 400, "white"),
                                     ("Now the shortest-path robot, which ignores hazards, on the identical scenario.", 38, REG, 530, "0x9fb0c3")], 4))
    for j, (m0, m1, gb, ga) in enumerate(cmp):
        a0, b0 = main_t.video_at_mission(m0), main_t.video_at_mission(m1)
        a1 = base_t.video_at_mission(m0)
        cap = f"Mission time {m0:.0f}-{m1:.0f} s. Hazard exposure in this window: AegisNav {ga:.1f}, shortest-path robot {gb:.1f} risk·s."
        parts.append(clip(work, 500 + j, main_t, a0, b0, 3.0, cap, take_other=base_t, other_start=a1,
                          labels=("AegisNav (onboard sensing)", "Shortest-path baseline")))
    parts.append(still(work, "results", Path(args.slide), 9))
    parts.append(card(work, "end", [("What this is, and is not", 56, BOLD, 150, "white"),
                                     ("Simulation only (ROS 2 Jazzy, Gazebo Harmonic, Nav2). Not certified rescue equipment.", 32, REG, 300, "0x9fb0c3"),
                                     ("Fire, gas and smoke come from a synthetic forecast model, not fire physics.", 32, REG, 350, "0x9fb0c3"),
                                     ("The sensor and perception confidence are simulated models, not calibrated measurements.", 32, REG, 400, "0x9fb0c3"),
                                     ("Results are from a small number of matched simulated runs, not statistical proof.", 32, REG, 450, "0x9fb0c3"),
                                     ("Code, tests, decisions and every run's data: the repository (docs/, artifacts/batch/).", 30, REG, 600, "white"),
                                     ("3D models: Gazebo Fuel, Open Robotics (CC0). Fog Emitter texture: Open Robotics (CC BY 4.0).", 24, REG, 760, "0x7d8fa3")], 9))
    lst = work / "list.txt"
    lst.write_text("".join(f"file '{p.resolve()}'\n" for p in parts))
    run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c:v", "libx264", "-preset", "medium",
         "-crf", "19", "-pix_fmt", "yuv420p", "-r", "30", "-c:a", "aac", "-movflags", "+faststart", args.out])
    print("wrote", args.out)


if __name__ == "__main__":
    main()
