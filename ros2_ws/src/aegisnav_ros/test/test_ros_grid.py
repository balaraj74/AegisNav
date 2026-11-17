"""ROS-message adapter contract tests using lightweight fake message objects.

These tests intentionally do not import rclpy; they can run on machines where
ROS 2 is not installed and still catch geometry/contract regressions.
"""
from __future__ import annotations

from types import SimpleNamespace as NS
import unittest

from aegisnav_ros.ros_grid import (
    grid_from_occupancy_grid, observation_data, observation_from_message, origin_yaw, risk_frames_from_message,
    snap_to_free_cell, world_to_cell,
)
from aegisnav_core.models import Cell


def quaternion(z: float = 0.0, w: float = 1.0, x: float = 0.0, y: float = 0.0):
    return NS(x=x, y=y, z=z, w=w)


def origin(x: float = -1.0, y: float = 2.0, orientation=None):
    return NS(position=NS(x=x, y=y, z=0.0), orientation=orientation or quaternion())


def occupancy_grid(
    width: int = 3,
    height: int = 2,
    data=None,
    resolution: float = 0.5,
    map_origin=None,
    frame_id: str = "map",
):
    if data is None:
        data = [0] * (width * height)
    info = NS(width=width, height=height, resolution=resolution, origin=map_origin or origin())
    return NS(header=NS(frame_id=frame_id), info=info, data=list(data))


def hazard_forecast(grid, risk=(0, 10, 20, 30, 40, 50), uncertainty=(0, 10, 20, 30, 40, 50), frame_id="map"):
    def layer(values):
        return occupancy_grid(
            grid.width,
            grid.height,
            values,
            grid.resolution_m,
            origin(grid.origin_x_m, grid.origin_y_m),
            frame_id,
        )
    return NS(
        header=NS(frame_id=frame_id),
        time_step_s=0.5,
        risk_layers=[layer(risk), layer(risk)],
        uncertainty_layers=[layer(uncertainty), layer(uncertainty)],
    )


class RosGridAdapterTests(unittest.TestCase):
    def test_grid_conversion_treats_unknown_and_occupied_as_blocked(self):
        msg = occupancy_grid(data=[0, 49, 50, -1, 0, 100])
        grid = grid_from_occupancy_grid(msg)
        self.assertEqual(grid.width, 3)
        self.assertEqual(grid.height, 2)
        self.assertEqual(grid.resolution_m, 0.5)
        self.assertEqual(grid.origin_x_m, -1.0)
        self.assertEqual(grid.origin_y_m, 2.0)
        self.assertEqual(grid.blocked, frozenset({2, 3, 5}))

    def test_world_to_cell_uses_map_origin_and_half_open_bounds(self):
        grid = grid_from_occupancy_grid(occupancy_grid())
        self.assertEqual(world_to_cell(grid, -0.75, 2.25), Cell(0, 0))
        self.assertEqual(world_to_cell(grid, 0.49, 2.99), Cell(2, 1))
        self.assertIsNone(world_to_cell(grid, 0.5, 2.5))
        self.assertIsNone(world_to_cell(grid, -1.01, 2.2))
        with self.assertRaisesRegex(ValueError, "must be finite"):
            world_to_cell(grid, float("nan"), 2.2)

    def test_origin_yaw_rejects_non_normalized_quaternion(self):
        with self.assertRaisesRegex(ValueError, "quaternion norm"):
            origin_yaw(quaternion(z=0.0, w=2.0))

    def test_rotated_map_is_rejected_explicitly(self):
        half = 0.3826834323650898  # sin(45 degrees / 2)
        with self.assertRaisesRegex(ValueError, "Rotated OccupancyGrid"):
            grid_from_occupancy_grid(occupancy_grid(map_origin=origin(orientation=quaternion(z=half, w=0.9238795325112867))))

    def test_forecast_layers_convert_to_normalized_frames(self):
        grid = grid_from_occupancy_grid(occupancy_grid())
        msg = hazard_forecast(grid, risk=(0, 10, 20, 30, 40, 100), uncertainty=(100, 40, 30, 20, 10, 0))
        frames = risk_frames_from_message(msg, grid, "map")
        self.assertEqual(len(frames), 2)
        self.assertAlmostEqual(frames[0].risk[1], 0.1)
        self.assertAlmostEqual(frames[0].risk[-1], 1.0)
        self.assertAlmostEqual(frames[0].uncertainty[0], 1.0)
        self.assertEqual(frames[1].step_index, 1)
        self.assertAlmostEqual(frames[1].time_s, 0.5)

    def test_forecast_rejects_mismatched_layer_counts(self):
        grid = grid_from_occupancy_grid(occupancy_grid())
        msg = hazard_forecast(grid)
        msg.uncertainty_layers = msg.uncertainty_layers[:1]
        with self.assertRaisesRegex(ValueError, "matching set"):
            risk_frames_from_message(msg, grid, "map")

    def test_forecast_rejects_frame_mismatch(self):
        grid = grid_from_occupancy_grid(occupancy_grid())
        msg = hazard_forecast(grid, frame_id="odom")
        with self.assertRaisesRegex(ValueError, "does not match map frame"):
            risk_frames_from_message(msg, grid, "map")

    def test_forecast_rejects_geometry_mismatch(self):
        grid = grid_from_occupancy_grid(occupancy_grid())
        msg = hazard_forecast(grid)
        msg.risk_layers[0].info.resolution = 0.75
        with self.assertRaisesRegex(ValueError, "resolution does not match"):
            risk_frames_from_message(msg, grid, "map")


    def test_grid_conversion_rejects_short_data(self):
        with self.assertRaisesRegex(ValueError, "does not match dimensions"):
            grid_from_occupancy_grid(occupancy_grid(data=[0, 0]))

    def test_grid_conversion_rejects_invalid_occupancy_values(self):
        with self.assertRaisesRegex(ValueError, "valid range"):
            grid_from_occupancy_grid(occupancy_grid(data=[0, 0, 101, 0, 0, 0]))

    def test_grid_conversion_rejects_invalid_occupied_threshold(self):
        with self.assertRaisesRegex(ValueError, "occupied_threshold"):
            grid_from_occupancy_grid(occupancy_grid(), occupied_threshold=0)

    def test_grid_conversion_rejects_roll_pitch_rotation(self):
        with self.assertRaisesRegex(ValueError, "Roll/pitch-rotated"):
            grid_from_occupancy_grid(occupancy_grid(map_origin=origin(orientation=quaternion(x=0.1, w=0.99498743710662))))

    def test_forecast_rejects_missing_frame_and_invalid_time_step(self):
        grid = grid_from_occupancy_grid(occupancy_grid())
        missing_frame = hazard_forecast(grid, frame_id="")
        with self.assertRaisesRegex(ValueError, "does not match map frame"):
            risk_frames_from_message(missing_frame, grid, "map")
        invalid_step = hazard_forecast(grid)
        invalid_step.time_step_s = 0.0
        with self.assertRaisesRegex(ValueError, "time_step_s"):
            risk_frames_from_message(invalid_step, grid, "map")

    def test_forecast_rejects_short_layer_data(self):
        grid = grid_from_occupancy_grid(occupancy_grid())
        msg = hazard_forecast(grid)
        msg.risk_layers[0].data = [0]
        with self.assertRaisesRegex(ValueError, "data length"):
            risk_frames_from_message(msg, grid, "map")


class SnapTests(unittest.TestCase):
    def setUp(self):
        from aegisnav_core.models import GridMap
        # 3x1 strip, 0.4 m cells, origin 0; cell 1 is a wall.
        self.grid = GridMap(3, 1, 0.4, frozenset({1}))

    def test_free_cell_is_returned_unchanged(self):
        self.assertEqual(snap_to_free_cell(self.grid, 0.2, 0.2, 0.25), Cell(0, 0))

    def test_pose_just_inside_wall_snaps_to_nearest_free_cell(self):
        self.assertEqual(snap_to_free_cell(self.grid, 0.41, 0.2, 0.25), Cell(0, 0))
        self.assertEqual(snap_to_free_cell(self.grid, 0.79, 0.2, 0.25), Cell(2, 0))

    def test_pose_deep_in_wall_fails_closed(self):
        self.assertIsNone(snap_to_free_cell(self.grid, 0.6, 0.2, 0.15))

    def test_invalid_tolerance(self):
        with self.assertRaises(ValueError):
            snap_to_free_cell(self.grid, 0.2, 0.2, -1.0)


class ObservationAdapterTests(unittest.TestCase):
    def setUp(self):
        self.grid = grid_from_occupancy_grid(occupancy_grid())

    def obs(self, data, **kw):
        g = self.grid
        return occupancy_grid(g.width, g.height, data, kw.pop("resolution", g.resolution_m),
                              origin(g.origin_x_m, g.origin_y_m), kw.pop("frame_id", "map"))

    def test_round_trip_keeps_unseen_cells_unseen(self):
        data = observation_data({0: 0.9, 4: 0.0}, self.grid.size)
        self.assertEqual(data, [90, -1, -1, -1, 0, -1])
        self.assertEqual(observation_from_message(self.obs(data), self.grid, "map"), {0: 0.9, 4: 0.0})

    def test_misaligned_or_malformed_observation_fails_closed(self):
        for bad in (self.obs([0] * 5), self.obs([0] * 6, resolution=1.0), self.obs([0] * 6, frame_id="odom"),
                    self.obs([101, 0, 0, 0, 0, 0])):
            with self.assertRaises(ValueError):
                observation_from_message(bad, self.grid, "map")
        with self.assertRaises(ValueError):
            observation_data({9: 0.5}, self.grid.size)
        with self.assertRaises(ValueError):
            observation_data({0: 1.2}, self.grid.size)


if __name__ == "__main__":
    unittest.main()
