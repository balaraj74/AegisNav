"""Dependency-free structural checks for the source archive."""
from __future__ import annotations

from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
errors: list[str] = []

required = [
    "AGENTS.md", "agent.md", ".agent/README.md", ".agent/rules/10-no-vibecoding.md",
    "docs/architecture/system_overview.md", "docs/architecture/feature_direction.md",
    "docs/research/research_position.md", "CONTRIBUTING.md", ".github/workflows/quality.yml",
    "ros2_ws/src/aegisnav_core/aegisnav_core/planning.py",
    "ros2_ws/src/aegisnav_ros/aegisnav_ros/planner_node.py",
    "ros2_ws/src/aegisnav_msgs/msg/HazardForecast.msg",
    "ros2_ws/src/aegisnav_bringup/worlds/aegisnav_rescue.sdf",
    "ros2_ws/src/aegisnav_bringup/maps/aegisnav_rescue.yaml",
]
for relative in required:
    if not (ROOT / relative).is_file():
        errors.append(f"Missing required file: {relative}")

for path in [*ROOT.glob("ros2_ws/src/**/package.xml"), *ROOT.glob("ros2_ws/src/**/*.sdf")]:
    try:
        ET.parse(path)
    except (ET.ParseError, OSError) as exc:
        errors.append(f"Invalid XML {path.relative_to(ROOT)}: {exc}")

pgm = ROOT / "ros2_ws/src/aegisnav_bringup/maps/aegisnav_rescue.pgm"
if pgm.exists():
    lines = pgm.read_text(encoding="ascii").splitlines()
    data_lines = [line for line in lines if line.strip() and not line.lstrip().startswith("#")]
    if len(data_lines) < 4 or data_lines[0].strip() != "P2":
        errors.append("Sample map must be an ASCII P2 PGM")
    else:
        try:
            width, height = (int(v) for v in data_lines[1].split())
            max_value = int(data_lines[2])
            pixels = [int(v) for line in data_lines[3:] for v in line.split()]
            if max_value != 255 or len(pixels) != width * height:
                errors.append(f"PGM pixel count/header mismatch: expected {width*height}, found {len(pixels)}")
            if (width, height) != (24, 16):
                errors.append(f"Unexpected sample map dimensions: {width}x{height}")
        except Exception as exc:
            errors.append(f"Could not validate PGM header/data: {exc}")

world = ROOT / "ros2_ws/src/aegisnav_bringup/worlds/aegisnav_rescue.sdf"
if world.exists():
    content = world.read_text(encoding="utf-8")
    for token in ("aegis_bot", "gz-sim-diff-drive-system", "front_lidar", "aegisnav:decor:begin", "visibility_mask"):
        if token not in content:
            errors.append(f"Sample SDF missing expected scene token: {token}")

# Catch accidental absolute home paths and obvious API-key-like literals in code.
code_extensions = {".py", ".yaml", ".yml", ".json", ".sh"}
for path in ROOT.rglob("*"):
    if not path.is_file() or path.suffix not in code_extensions or any(part in {".git", "build", "install", "log", "__pycache__"} for part in path.parts) \
            or path.relative_to(ROOT).as_posix() == ".claude/settings.local.json":  # per-user tool settings, git-ignored
        continue
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        continue
    if re.search(r"/home/[A-Za-z0-9_-]+/", text):
        errors.append(f"Machine-specific home path found: {path.relative_to(ROOT)}")
    if re.search(r"(?i)(api[_-]?key|secret[_-]?key)\s*[:=]\s*['\"]?[A-Za-z0-9_\-]{24,}", text):
        errors.append(f"Possible secret-like literal found: {path.relative_to(ROOT)}")

if errors:
    print("REPOSITORY CHECK FAILED", file=sys.stderr)
    for error in errors:
        print(f" - {error}", file=sys.stderr)
    sys.exit(1)

print("Repository structural checks passed.")
print(" - required architecture/agent files present")
print(" - package.xml and SDF XML parse")
print(" - sample PGM dimensions and pixel count validated")
print(" - expected robot, lidar and hazard anchors present")
print(" - no obvious machine-specific path or secret literal detected")
