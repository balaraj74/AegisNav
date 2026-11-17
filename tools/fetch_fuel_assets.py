"""Fetch the pinned, permissively licensed Gazebo Fuel models used by the rescue_building world.

Network is needed only here (never at simulation runtime). For every model the script
  * downloads the pinned version from fuel.gazebosim.org,
  * checks the license reported by Fuel's own metadata is in ALLOWED_LICENSES (refuses otherwise),
  * records an attribution line in the lock file for licenses that require one (CC BY 4.0),
  * checks the archive SHA-256 against tools/fuel_assets.lock.json (or records it with --write-lock),
  * extracts it to ros2_ws/src/aegisnav_bringup/models/<dir>, the directory name its mesh URIs expect,
  * removes <include> elements that point to the internet (they would download at runtime),
  * strips whitespace before the XML declaration (invalid XML; Gazebo tolerates it, parsers do not).
Usage: python3 tools/fetch_fuel_assets.py [--write-lock] [--only "Fuel name"]
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import re
import shutil
import sys
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = ROOT / "ros2_ws/src/aegisnav_bringup/models"
LOCK = ROOT / "tools/fuel_assets.lock.json"
# Fuel license name -> whether redistribution requires attribution. Anything else is refused.
ALLOWED_LICENSES = {
    "Creative Commons Zero v1.0 Universal": False,
    "Creative Commons Attribution 4.0 International": True,
}

# (Fuel name, pinned version, local directory = name used by the model's own mesh URIs)
MODELS = [
    ("Clearpath Robotics Husky A200", 3, "husky"),
    ("Rescue Randy Sitting", 8, "rescue_randy_sitting"),
    ("Collapsed House", 4, "collapsed_house"),
    ("Collapsed Fire Station", 3, "collapsed_fire_station"),
    ("Collapsed industrial building", 3, "collapsed_industrial"),
    ("Collapsed police station", 3, "collapsed_police_station"),
    ("Fire truck", 3, "fire_truck"),
    ("Construction Cone", 3, "construction_cone"),
    ("Construction Barrel", 4, "construction_barrel"),
    ("Cinder Block", 3, "cinder_block"),
    ("Jersey Barrier", 6, "jersey_barrier"),
    ("Euro pallet", 3, "euro_pallet"),
    ("Fire Extinguisher", 5, "fire_extinguisher"),
    ("Fire hydrant", 3, "fire_hydrant"),
    ("Fog Emitter", 5, "fog_emitter"),  # CC BY 4.0: textures reused by the hazard fire/smoke effect
]
TEXTURE_REF = re.compile(r"(<init_from>|map_Kd\s+)([^<\s/]+\.(?:png|jpg|jpeg))")
EXTERNAL_INCLUDE = re.compile(r"\s*<include>\s*<uri>https?://[^<]*</uri>\s*</include>", re.S)


def fetch(name: str, version: int) -> tuple[bytes, dict]:
    q = urllib.parse.quote(name)
    # Fuel exposes license metadata per model (no per-version endpoint); the archive below is the pinned version.
    meta = json.load(urllib.request.urlopen(f"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{q}", timeout=30))
    data = urllib.request.urlopen(f"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{q}/{version}/{q}.zip", timeout=120).read()
    return data, meta


def attribution(name: str, version: int, meta: dict, model_dir: Path) -> str:
    """Credit line: title, author(s) from model.config, Fuel owner, source URL and license."""
    config = model_dir / "model.config"
    root = ET.parse(config).getroot() if config.exists() else None
    authors = [a.text.strip() for a in root.findall("author/name") if a.text] if root is not None else []
    by = ", ".join(authors) or str(meta.get("owner"))
    url = f"https://app.gazebosim.org/{meta.get('owner')}/fuel/models/{urllib.parse.quote(name)}/{version}"
    return f'"{name}" v{version} by {by} ({meta.get("owner")}), {url}, licensed {meta.get("license_name")} ({meta.get("license_url")})'


def _localize_textures(model_dir: Path) -> None:
    """Meshes name textures by bare filename; point them at materials/textures/ when the file is there."""
    textures = model_dir / "materials" / "textures"
    for mesh in [*model_dir.glob("meshes/*.dae"), *model_dir.glob("meshes/*.mtl")]:
        text = mesh.read_text(encoding="utf-8", errors="ignore")
        fixed = TEXTURE_REF.sub(
            lambda m: m.group(1) + ("../materials/textures/" + m.group(2) if (textures / m.group(2)).exists() else m.group(2)),
            text)
        if fixed != text:
            mesh.write_text(fixed, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write-lock", action="store_true", help="record archive hashes instead of verifying them")
    ap.add_argument("--only", action="append", default=[], metavar="NAME", help="fetch only this Fuel model (repeatable)")
    args = ap.parse_args()
    lock = json.loads(LOCK.read_text()) if LOCK.exists() else {}
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    unknown = set(args.only) - {m[0] for m in MODELS}
    if unknown:
        print(f"not in MODELS: {sorted(unknown)}", file=sys.stderr)
        return 1
    for name, version, directory in MODELS:
        if args.only and name not in args.only:
            continue
        data, meta = fetch(name, version)
        license_name = str(meta.get("license_name"))
        if license_name not in ALLOWED_LICENSES:
            print(f"REFUSED {name} v{version}: license is {license_name!r}", file=sys.stderr)
            return 1
        digest = hashlib.sha256(data).hexdigest()
        key = f"{name}@{version}"
        if not args.write_lock and lock.get(key, {}).get("sha256") != digest:
            print(f"HASH MISMATCH for {key}: got {digest}", file=sys.stderr)
            return 1
        target = MODELS_DIR / directory
        if target.exists():
            shutil.rmtree(target)
        zipfile.ZipFile(io.BytesIO(data)).extractall(target)
        for sdf in target.glob("*.sdf"):
            text = sdf.read_text(encoding="utf-8")
            cleaned = EXTERNAL_INCLUDE.sub("", text).lstrip()
            if cleaned != text:
                sdf.write_text(cleaned, encoding="utf-8")
        _localize_textures(target)
        if args.write_lock:
            lock[key] = {"sha256": digest, "license": license_name, "dir": directory}
            if ALLOWED_LICENSES[license_name]:
                lock[key]["attribution"] = attribution(name, version, meta, target)
        print(f"ok {key} -> models/{directory} ({len(data) // 1024} KiB, {license_name})")
    if args.write_lock:
        LOCK.write_text(json.dumps(lock, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
