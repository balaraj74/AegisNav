"""Cross-checks between the core scenario grid, the PGM map, the SDF world and the Nav2/AegisNav configs.

Source-only: parses files, needs no ROS or Gazebo. These catch misaligned maps before a live run;
they do not replace one.
"""
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from aegisnav_core.models import Cell
from aegisnav_core.scenarios import corridor_scenario

ROOT = Path(__file__).resolve().parents[1]
ROS_CFG = ROOT.parent / "aegisnav_ros" / "config" / "aegisnav.yaml"


def read_pgm(path: Path):
    tokens = [line for line in path.read_text(encoding="ascii").splitlines() if not line.lstrip().startswith("#")]
    words = " ".join(tokens).split()
    assert words[0] == "P2"
    w, h, _max = int(words[1]), int(words[2]), int(words[3])
    pix = [int(v) for v in words[4:]]
    assert len(pix) == w * h
    return w, h, pix


def yaml_scalar(text: str, key: str) -> str:
    match = re.search(rf"^\s*{re.escape(key)}:\s*(.+?)\s*(?:#.*)?$", text, re.M)
    assert match, f"{key} not found"
    return match.group(1)


def world_boxes(world: str = "aegisnav_rescue.sdf"):
    """(cx, cy, sx, sy) for every static, collision-bearing, unrotated `wall_*` box model."""
    boxes = []
    for model in ET.parse(ROOT / "worlds" / world).getroot().iter("model"):
        name = model.get("name", "")
        if not name.startswith("wall_"):
            continue
        pose = [float(v) for v in model.find("pose").text.split()]
        assert pose[3:] == [0.0, 0.0, 0.0], f"{name} is rotated; this check assumes axis-aligned walls"
        size = [float(v) for v in model.find(".//collision//size").text.split()]
        boxes.append((pose[0], pose[1], size[0], size[1]))
    return boxes


def dae_bounds(path: Path, scale: float = 1.0):
    """Axis-aligned (min, max) xyz of a COLLADA mesh in model coordinates: node matrices and <unit> applied."""
    t = ET.parse(path).getroot()
    ns = {"c": t.tag.split("}")[0].strip("{")}
    unit = t.find(".//c:asset/c:unit", ns)
    scale *= float(unit.get("meter", "1")) if unit is not None else 1.0
    geoms = {}
    for g in t.iterfind(".//c:geometry", ns):
        pts = []
        for src in g.iterfind(".//c:source", ns):
            if "pos" in src.get("id", "").lower():
                v = [float(x) for x in src.find("c:float_array", ns).text.split()]
                pts += list(zip(v[0::3], v[1::3], v[2::3]))
        geoms["#" + g.get("id")] = pts
    lo, hi = [float("inf")] * 3, [float("-inf")] * 3

    def walk(node, m):
        mat = node.find("c:matrix", ns)
        if mat is not None:
            a = [float(x) for x in mat.text.split()]
            m = [[sum(m[i][k] * a[k * 4 + j] for k in range(4)) for j in range(4)] for i in range(4)]
        for inst in node.iterfind("c:instance_geometry", ns):
            for x, y, z in geoms.get(inst.get("url"), []):
                for i in range(3):
                    v = (m[i][0] * x + m[i][1] * y + m[i][2] * z + m[i][3]) * scale
                    lo[i], hi[i] = min(lo[i], v), max(hi[i], v)
        for child in node.iterfind("c:node", ns):
            walk(child, m)

    for vs in t.iterfind(".//c:visual_scene", ns):
        for n in vs.iterfind("c:node", ns):
            walk(n, [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)])
    return lo, hi


class MapAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.scn = corridor_scenario()
        self.grid = self.scn.grid

    def test_map_yaml_matches_grid_geometry(self):
        text = (ROOT / "maps" / "aegisnav_rescue.yaml").read_text()
        self.assertAlmostEqual(float(yaml_scalar(text, "resolution")), self.grid.resolution_m)
        origin = [float(v) for v in yaml_scalar(text, "origin").strip("[]").split(",")]
        self.assertAlmostEqual(origin[0], self.grid.origin_x_m)
        self.assertAlmostEqual(origin[1], self.grid.origin_y_m)

    def test_pgm_obstacles_equal_core_grid_with_y_flip(self):
        w, h, pix = read_pgm(ROOT / "maps" / "aegisnav_rescue.pgm")
        self.assertEqual((w, h), (self.grid.width, self.grid.height))
        blocked = {(c % w, h - 1 - (c // w)) for c, v in enumerate(pix) if v < 128}
        expected = {(self.grid.cell(i).x, self.grid.cell(i).y) for i in self.grid.blocked}
        self.assertEqual(blocked, expected)

    def test_fine_localization_map_is_planning_grid_upsampled(self):
        w, h, pix = read_pgm(ROOT / "maps" / "aegisnav_rescue_fine.pgm")
        k = w // self.grid.width
        self.assertEqual((w, h, k * self.grid.height), (self.grid.width * k, h, h))
        text = (ROOT / "maps" / "aegisnav_rescue_fine.yaml").read_text()
        self.assertAlmostEqual(float(yaml_scalar(text, "resolution")) * k, self.grid.resolution_m)
        for i, v in enumerate(pix):
            px, py = i % w, h - 1 - i // w
            self.assertEqual(v < 128, self.grid.is_blocked(Cell(px // k, py // k)), (px, py))

    def test_sdf_walls_cover_exactly_the_blocked_cells(self):
        boxes = world_boxes()
        for idx in range(self.grid.size):
            cell = self.grid.cell(idx)
            x, y = self.grid.cell_center_world(cell)
            covered = any(abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2 for cx, cy, sx, sy in boxes)
            self.assertEqual(covered, idx in self.grid.blocked, f"cell {cell} wall/grid mismatch")

    def test_robot_spawn_is_in_start_cell_and_free(self):
        robot = next(m for m in ET.parse(ROOT / "worlds" / "aegisnav_rescue.sdf").getroot().iter("model") if m.get("name") == "aegis_bot")
        x, y = (float(v) for v in robot.find("pose").text.split()[:2])
        cx = int((x - self.grid.origin_x_m) // self.grid.resolution_m)
        cy = int((y - self.grid.origin_y_m) // self.grid.resolution_m)
        self.assertEqual(Cell(cx, cy), self.scn.start)
        self.assertFalse(self.grid.is_blocked(Cell(cx, cy)))

    def test_robot_rests_level(self):
        # Observed in Gazebo: a caster 15 mm short pitched the robot 7.6 deg and the lidar
        # plane hit the floor, filling the local costmap with phantom obstacles.
        robot = next(m for m in ET.parse(ROOT / "worlds" / "aegisnav_rescue.sdf").getroot().iter("model") if m.get("name") == "aegis_bot")
        bottoms = {}
        for link in robot.iter("link"):
            pose = link.find("pose")
            z = float(pose.text.split()[2]) if pose is not None else 0.0
            geom = link.find("collision/geometry")
            if geom.find("cylinder") is not None:  # wheels (axis horizontal)
                bottoms[link.get("name")] = z - float(geom.find("cylinder/radius").text)
            elif geom.find("sphere") is not None:
                bottoms[link.get("name")] = z - float(geom.find("sphere/radius").text)
        self.assertEqual(set(bottoms), {"left_wheel", "right_wheel", "rear_caster", "front_caster"})
        for name, bottom in bottoms.items():
            self.assertAlmostEqual(bottom, bottoms["left_wheel"], places=3, msg=name)

    def test_casters_support_both_ends(self):
        # Observed in Gazebo: with mass over the axle and only a rear caster, braking tipped the
        # nose ~27 deg into the floor and the lidar reported phantom obstacles ~0.37 m ahead.
        robot = next(m for m in ET.parse(ROOT / "worlds" / "aegisnav_rescue.sdf").getroot().iter("model") if m.get("name") == "aegis_bot")
        xs = {l.get("name"): float(l.find("pose").text.split()[0]) for l in robot.iter("link") if "caster" in l.get("name")}
        self.assertTrue(min(xs.values()) < 0 < max(xs.values()), xs)

    def test_wheel_joints_spin_about_model_y_at_wheel_centres(self):
        # Observed in Gazebo: <xyz>0 1 0</xyz> in the rolled wheel frame made the spin axis vertical;
        # commanded forward motion drove the robot sideways and pitched it 43 deg.
        robot = next(m for m in ET.parse(ROOT / "worlds" / "aegisnav_rescue.sdf").getroot().iter("model") if m.get("name") == "aegis_bot")
        links = {l.get("name"): l for l in robot.iter("link")}
        for side in ("left", "right"):
            joint = next(j for j in robot.iter("joint") if j.get("name") == f"{side}_wheel_joint")
            xyz = joint.find("axis/xyz")
            self.assertEqual((xyz.get("expressed_in"), xyz.text.split()), ("__model__", ["0", "1", "0"]))
            self.assertEqual(joint.find("pose").get("relative_to"), "base_link")
            self.assertEqual(joint.find("pose").text.split()[:3], links[f"{side}_wheel"].find("pose").text.split()[:3])

    def test_world_copy_in_assets_is_identical(self):
        a = ROOT / "worlds" / "aegisnav_rescue.sdf"
        b = ROOT.parents[2] / "assets" / "worlds" / "aegisnav_rescue.sdf"
        self.assertEqual(a.read_text(), b.read_text())


class ConfigConsistencyTests(unittest.TestCase):
    def test_planner_speed_matches_controller_max_speed(self):
        planner = float(yaml_scalar(ROS_CFG.read_text(), "nominal_speed_mps"))
        nav2 = (ROOT / "config" / "nav2_params.yaml").read_text()
        self.assertAlmostEqual(float(yaml_scalar(nav2, "desired_linear_vel")), planner)

    def test_mission_cells_are_free_in_scenario_grid(self):
        scn = corridor_scenario()
        text = ROS_CFG.read_text()
        base = [int(v) for v in yaml_scalar(text, "base_cell_xy").strip("[]").split(",")]
        refuges = [int(v) for v in yaml_scalar(text, "refuge_cells_xy").strip("[]").split(",")]
        self.assertEqual(Cell(*base), scn.base)
        self.assertEqual([Cell(refuges[i], refuges[i + 1]) for i in range(0, len(refuges), 2)], list(scn.refuges))

    def test_every_world_preset_is_consistent(self):
        import yaml
        presets = yaml.safe_load((ROOT / "config" / "worlds.yaml").read_text())
        from aegisnav_core.scenarios import scenario_by_name
        for name, p in presets.items():
            nav2 = (ROOT / p["nav2_params"]).read_text()
            self.assertAlmostEqual(float(yaml_scalar(nav2, "desired_linear_vel")), float(p["nominal_speed_mps"]), msg=name)
            scn = scenario_by_name(p["scenario_name"])
            g = scn.grid
            sx, sy = p["spawn_xy"]
            cell = Cell(int((sx - g.origin_x_m) // g.resolution_m), int((sy - g.origin_y_m) // g.resolution_m))
            self.assertEqual(cell, scn.start, name)
            for key in ("world_sdf", "planning_map", "localization_map", "nav2_params"):
                self.assertTrue((ROOT / p[key]).is_file(), (name, key))
            amcl = re.search(r"initial_pose:\s*\n\s*x:\s*(-?[0-9.]+)\s*\n\s*y:\s*(-?[0-9.]+)", nav2)
            self.assertEqual((float(amcl.group(1)), float(amcl.group(2))), (sx, sy), name)

    def test_generated_worlds_and_maps_match_their_scenario(self):
        # Every fully generated preset world (rescue_building, rescue_compound): maps and SDF walls
        # agree with the core grid cell for cell; props and victims are drawn as models, not walls.
        import yaml
        from aegisnav_core.scenarios import scenario_by_name
        presets = yaml.safe_load((ROOT / "config" / "worlds.yaml").read_text())
        for name, p in presets.items():
            if p["scenario_name"] == "corridor":
                continue  # covered by MapAlignmentTests
            scn = scenario_by_name(p["scenario_name"])
            g = scn.grid
            w, h, pix = read_pgm(ROOT / p["planning_map"].replace(".yaml", ".pgm"))
            self.assertEqual((w, h), (g.width, g.height), name)
            self.assertEqual({(i % w, h - 1 - i // w) for i, v in enumerate(pix) if v < 128},
                             {(g.cell(i).x, g.cell(i).y) for i in g.blocked}, name)
            fw, fh, fpix = read_pgm(ROOT / p["localization_map"].replace(".yaml", ".pgm"))
            k = fw // g.width
            self.assertEqual((fw, fh), (g.width * k, g.height * k), name)
            for i in range(0, len(fpix), 7):  # sampled: the full fine map is ~1M pixels
                px, py = i % fw, fh - 1 - i // fw
                self.assertEqual(fpix[i] < 128, g.is_blocked(Cell(px // k, py // k)), (name, px, py))
            drawn = {c for c, _ in scn.props} | set(scn.victims)
            boxes = world_boxes(Path(p["world_sdf"]).name)
            for idx in range(g.size):
                cell = g.cell(idx)
                x, y = g.cell_center_world(cell)
                covered = any(abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2 for cx, cy, sx, sy in boxes)
                self.assertEqual(covered, idx in g.blocked and cell not in drawn, (name, cell))
            robot = next(m for m in ET.parse(ROOT / p["world_sdf"]).getroot().iter("model") if m.get("name") == "aegis_bot")
            rx, ry = (float(v) for v in robot.find("pose").text.split()[:2])
            self.assertEqual(Cell(int((rx - g.origin_x_m) // g.resolution_m), int((ry - g.origin_y_m) // g.resolution_m)),
                             scn.start, name)

    def test_mesh_footprints_used_by_the_generator_match_the_meshes(self):
        import sys
        sys.path.insert(0, str(ROOT.parents[2] / "tools"))
        from world_rescue_building import PALLET_HALF_HEIGHT_M, PALLET_HALF_SIZE_M, RANDY_FOOTPRINT_M
        lo, hi = dae_bounds(ROOT / "models" / "euro_pallet" / "meshes" / "pallet.dae", 0.1)  # model.sdf <scale> 0.1
        self.assertAlmostEqual(-lo[0], PALLET_HALF_SIZE_M[0], delta=0.01)
        self.assertAlmostEqual(hi[0], PALLET_HALF_SIZE_M[0], delta=0.01)
        self.assertAlmostEqual(-lo[1], PALLET_HALF_SIZE_M[1], delta=0.01)
        self.assertAlmostEqual(hi[1], PALLET_HALF_SIZE_M[1], delta=0.01)
        self.assertAlmostEqual((hi[2] - lo[2]) / 2, PALLET_HALF_HEIGHT_M, delta=0.01)
        lo, hi = dae_bounds(ROOT / "models" / "rescue_randy_sitting" / "meshes" / "rescue_randy.dae")
        for got, want in zip((lo[0], lo[1], hi[0], hi[1]), (*RANDY_FOOTPRINT_M[0], *RANDY_FOOTPRINT_M[1])):
            self.assertAlmostEqual(got, want, delta=0.01)

    def test_pallets_and_victims_stay_on_blocked_cells(self):
        # A prop larger than its blocked cells is an obstacle the planner does not know about (observed:
        # the Husky drove into a misplaced pallet stack in rescue_compound).
        import sys
        import yaml
        sys.path.insert(0, str(ROOT.parents[2] / "tools"))
        from world_rescue_building import PALLET_HALF_SIZE_M, RANDY_FOOTPRINT_M, _footprint_points
        from aegisnav_core.scenarios import scenario_by_name
        presets = yaml.safe_load((ROOT / "config" / "worlds.yaml").read_text())
        hx, hy = PALLET_HALF_SIZE_M
        for name, p in presets.items():
            if p["scenario_name"] == "corridor":
                continue
            g = scenario_by_name(p["scenario_name"]).grid
            r = g.resolution_m
            blocked = [g.cell(i) for i in g.blocked]
            for inc in ET.parse(ROOT / p["world_sdf"]).getroot().iter("include"):
                label = inc.find("name").text
                if not label.startswith(("pallet_", "victim_")):
                    continue
                x, y, _z, _r, _p, yaw = (float(v) for v in inc.find("pose").text.split())
                footprint = RANDY_FOOTPRINT_M if label.startswith("victim_") else ((-hx, -hy), (hx, hy))
                worst = 0.0
                for px, py in _footprint_points(x, y, yaw, footprint):
                    c = Cell(int((px - g.origin_x_m) // r), int((py - g.origin_y_m) // r))
                    if g.is_blocked(c):
                        continue
                    worst = max(worst, min(max(abs(px - (g.origin_x_m + (b.x + 0.5) * r)) - r / 2,
                                               abs(py - (g.origin_y_m + (b.y + 0.5) * r)) - r / 2, 0.0)
                                           for b in blocked if abs(b.x - c.x) <= 3 and abs(b.y - c.y) <= 3))
                limit = 0.1 if label.startswith("victim_") else 0.0
                self.assertLessEqual(worst, limit + 1e-6, (name, label, round(worst, 3)))

    def test_controller_ids_match_planner_defaults(self):
        nav2 = (ROOT / "config" / "nav2_params.yaml").read_text()
        node = (ROOT.parent / "aegisnav_ros" / "aegisnav_ros" / "planner_node.py").read_text()
        for plugin_key, param in (("controller_plugins", "controller_id"), ("goal_checker_plugins", "goal_checker_id"), ("progress_checker_plugins", "progress_checker_id")):
            declared = re.search(rf'{plugin_key}:\s*\["([^"]+)"\]', nav2).group(1)
            default = re.search(rf'declare_parameter\("{param}",\s*"([^"]+)"\)', node).group(1)
            self.assertEqual(declared, default)


if __name__ == "__main__":
    unittest.main()
