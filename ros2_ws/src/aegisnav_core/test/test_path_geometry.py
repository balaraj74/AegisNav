import unittest

from aegisnav_core.models import Cell, GridMap
from aegisnav_core.path_geometry import centred_waypoints


def gap_grid():
    # 5x3 grid, 1 m cells; row y=1 is a wall with a two-cell opening at x=1..2.
    g = GridMap(5, 3, 1.0)
    return GridMap(5, 3, 1.0, frozenset(g.index(Cell(x, 1)) for x in (0, 3, 4)))


class CentredWaypointTests(unittest.TestCase):
    def test_waypoint_in_two_cell_gap_moves_to_gap_centre(self):
        cells = (Cell(1, 0), Cell(1, 1), Cell(1, 2))
        pts = centred_waypoints(gap_grid(), cells)
        self.assertEqual(pts[1], (2.0, 1.5))  # boundary between the two free gap cells
        self.assertEqual(pts[0], (1.5, 0.5))  # endpoints stay at cell centres
        self.assertEqual(pts[2], (1.5, 2.5))

    def test_open_space_and_symmetric_walls_are_unchanged(self):
        g = GridMap(3, 3, 1.0)
        cells = (Cell(1, 0), Cell(1, 1), Cell(1, 2))
        self.assertEqual(centred_waypoints(g, cells), [g.cell_center_world(c) for c in cells])
        walls = GridMap(3, 3, 1.0, frozenset({g.index(Cell(0, 1)), g.index(Cell(2, 1))}))
        self.assertEqual(centred_waypoints(walls, cells)[1], (1.5, 1.5))

    def test_diagonal_entry_is_centred_on_straight_exit(self):
        # Diagonal into gap cell (2,1), then straight up: exit axis is y, so shift in x
        # away from the wall at (3,1) toward the free cell (1,1).
        g = gap_grid()
        cells = (Cell(1, 0), Cell(2, 1), Cell(2, 2))
        self.assertEqual(centred_waypoints(g, cells)[1], (2.0, 1.5))

    def test_purely_diagonal_waypoint_is_unchanged(self):
        g = GridMap(4, 4, 1.0)
        cells = (Cell(0, 0), Cell(1, 1), Cell(2, 2))
        self.assertEqual(centred_waypoints(g, cells)[1], g.cell_center_world(Cell(1, 1)))

    def test_shifted_points_never_enter_blocked_cells(self):
        from aegisnav_core.scenarios import corridor_scenario
        from aegisnav_core.forecast import HazardForecaster
        from aegisnav_core.planning import RiskAwareAStar
        s = corridor_scenario()
        fr = HazardForecaster().forecast(s.grid, s.initial_risk, s.initial_uncertainty, s.sources, s.forecast_config)
        plan = RiskAwareAStar().plan(s.grid, fr.frames, s.start, s.target)
        g = s.grid
        for x, y in centred_waypoints(g, plan.cells):
            for ex in (-1e-6, 1e-6):
                for ey in (-1e-6, 1e-6):
                    c = Cell(int((x + ex - g.origin_x_m) // g.resolution_m), int((y + ey - g.origin_y_m) // g.resolution_m))
                    if g.contains(c):
                        self.assertFalse(g.is_blocked(c), (x, y))


if __name__ == "__main__":
    unittest.main()


class DensifyTests(unittest.TestCase):
    def test_spacing_and_endpoints(self):
        from aegisnav_core.path_geometry import densify
        pts = densify([(0.0, 0.0), (0.4, 0.0), (0.4, 0.3)], 0.1)
        self.assertEqual(pts[0], (0.0, 0.0))
        self.assertEqual(pts[-1], (0.4, 0.3))
        self.assertEqual(len(pts), 1 + 4 + 3)
        for (a, b), (c, d) in zip(pts, pts[1:]):
            self.assertLessEqual(((c - a) ** 2 + (d - b) ** 2) ** 0.5, 0.1 + 1e-9)

    def test_invalid_step_and_empty(self):
        from aegisnav_core.path_geometry import densify
        self.assertEqual(densify([], 0.1), [])
        with self.assertRaises(ValueError):
            densify([(0, 0)], 0.0)
