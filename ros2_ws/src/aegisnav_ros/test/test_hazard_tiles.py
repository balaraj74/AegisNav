import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from aegisnav_core.models import Cell
from aegisnav_ros.hazard_tiles import (
    HAZARD_VISIBILITY_FLAGS, LIDAR_PLANE_Z_M, band_model_name, band_model_sdf, cells_by_band, desired_bands, risk_band,
)


class HazardTileTests(unittest.TestCase):
    def test_bands(self):
        self.assertEqual([risk_band(v) for v in (0.0, 0.2, 0.3, 0.5, 0.9, 1.0)], [0, 0, 1, 2, 3, 3])
        with self.assertRaises(ValueError):
            risk_band(1.2)

    def test_desired_skips_blocked_and_low(self):
        bands = desired_bands(2, [0.9, 0.2, 0.5, 0.9], frozenset({3}))
        self.assertEqual(bands, {Cell(0, 0): 3, Cell(0, 1): 2})

    def test_grouping_has_every_band(self):
        grouped = cells_by_band({Cell(0, 0): 3, Cell(1, 0): 3, Cell(2, 0): 1})
        self.assertEqual(grouped, {1: frozenset({Cell(2, 0)}), 2: frozenset(), 3: frozenset({Cell(0, 0), Cell(1, 0)})})

    def test_band_model_is_visual_only_flat_and_lidar_excluded(self):
        name = band_model_name(2, 7)
        model = ET.fromstring(band_model_sdf(name, 2, [(0.2, 0.2), (0.6, 0.2)], 0.36)).find("model")
        self.assertEqual(model.get("name"), "aegisnav_hazard_band2_g7")
        self.assertIsNone(model.find(".//collision"))
        visuals = model.findall(".//visual")
        self.assertEqual(len(visuals), 2)
        for v in visuals:
            self.assertEqual(v.find("geometry/box/size").text.split()[2], "0.010")
            self.assertEqual(int(v.find("visibility_flags").text), HAZARD_VISIBILITY_FLAGS)

    def test_effect_models(self):
        from aegisnav_ros.hazard_tiles import effect_model_sdf
        for kind, names in (("fire", {"flame", "smoke"}), ("gas", {"haze"})):
            model = ET.fromstring(effect_model_sdf(f"fx_{kind}_0", kind, models_dir="/m/")).find("model")
            self.assertEqual(model.find("static").text, "true")
            self.assertIsNone(model.find(".//collision"))
            emitters = list(model.iter("particle_emitter"))
            self.assertEqual({e.get("name") for e in emitters}, names)
            for e in emitters:  # no sensor scatter; textures from the bundled models, absolute paths
                self.assertEqual(float(e.find("particle_scatter_ratio").text), 0.0)
                self.assertEqual(e.find("emitting").text, "true")
                self.assertEqual(e.find("material/pbr/metal/albedo_map").text, "/m/fog_emitter/materials/textures/fog.png")
                self.assertTrue(e.find("color_range_image").text.startswith("/m/aegisnav_effects/materials/textures/"))
                self.assertGreater(float(e.find("lifetime").text), 0.0)
            visuals = list(model.iter("visual"))
            self.assertGreater(len(visuals), 0)
            for v in visuals:  # geometry under the particles: hidden from the lidar, translucent or glowing
                self.assertEqual(int(v.find("visibility_flags").text), HAZARD_VISIBILITY_FLAGS)
                self.assertTrue(0.0 <= float(v.find("transparency").text) < 1.0)
        with self.assertRaises(ValueError):
            effect_model_sdf("x", "water", models_dir="/m")

    def test_particles_stay_above_lidar_plane(self):
        # Observed: the gpu_lidar returns particles near its plane even with particle_scatter_ratio 0.
        from aegisnav_ros.hazard_tiles import _EMITTERS, PARTICLE_CLEARANCE_M
        for emitters in _EMITTERS.values():
            for name, z, _size, psize, _life, _rate, vmin, _vmax, scale, _ramp in emitters:
                self.assertGreaterEqual(z - psize / 2, LIDAR_PLANE_Z_M + PARTICLE_CLEARANCE_M, name)  # lowest edge at birth
                self.assertGreater(vmin, psize * scale / 2, name)  # rises faster than the quad grows down

    def test_lidar_plane_covers_every_world_lidar(self):
        # World height of each gpu_lidar = model z + link z + sensor z (robots are spawned level).
        worlds = Path(__file__).resolve().parents[2] / "aegisnav_bringup" / "worlds"
        if not worlds.is_dir():
            self.skipTest("source tree layout not available")
        heights = []
        for sdf in sorted(worlds.glob("*.sdf")):
            for model in ET.parse(sdf).getroot().iter("model"):
                for link in model.findall("link"):
                    for sensor in link.findall("sensor[@type='gpu_lidar']"):
                        z = sum(float((e.findtext("pose") or "0 0 0").split()[2]) for e in (model, link, sensor))
                        heights.append((sdf.name, z))
        self.assertGreaterEqual(len(heights), 2, heights)
        for name, z in heights:
            self.assertLessEqual(z, LIDAR_PLANE_Z_M + 1e-6, name)

    def test_effect_textures_exist(self):
        from pathlib import Path
        from aegisnav_ros.hazard_tiles import _EMITTERS
        models = Path(__file__).resolve().parents[2] / "aegisnav_bringup" / "models"
        if not models.is_dir():
            self.skipTest("source tree layout not available")
        self.assertTrue((models / "fog_emitter/materials/textures/fog.png").is_file())
        for emitters in _EMITTERS.values():
            for spec in emitters:
                self.assertTrue((models / "aegisnav_effects/materials/textures" / spec[-1]).is_file(), spec[-1])

    def test_fire_has_a_warm_glow_light_and_gas_does_not(self):
        from aegisnav_ros.hazard_tiles import effect_model_sdf
        fire = ET.fromstring(effect_model_sdf("fx_fire_0", "fire", models_dir="/m")).find("model")
        lights = list(fire.iter("light"))
        self.assertEqual(len(lights), 1)
        r, g, b, _a = (float(v) for v in lights[0].find("diffuse").text.split())
        self.assertTrue(r > g > b)  # warm
        self.assertEqual(lights[0].find("cast_shadows").text, "false")
        gas = ET.fromstring(effect_model_sdf("fx_gas_0", "gas", models_dir="/m")).find("model")
        self.assertEqual(list(gas.iter("light")), [])

    def test_invalid_arguments(self):
        with self.assertRaises(ValueError):
            band_model_sdf("x", 0, [], 0.36)
        with self.assertRaises(ValueError):
            band_model_sdf("x", 1, [], 0.0)


if __name__ == "__main__":
    unittest.main()
