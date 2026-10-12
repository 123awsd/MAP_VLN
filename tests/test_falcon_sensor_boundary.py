"""Flight polygons must not create lidar/camera obstacles."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_pcd_falcon import render_mid360


class SensorBoundaryTest(unittest.TestCase):
    def test_lidar_and_observed_free_are_independent_of_flight_polygon(self):
        occupancy = np.zeros((20, 60, 60), dtype=np.int8)
        occupancy[:, :, 45] = 100
        origin = np.array([-6., -6., -1.])
        position = np.array([0., 0., 1.1])
        all_allowed = np.ones((60, 60), dtype=bool)
        none_allowed = np.zeros_like(all_allowed)
        seen_a = np.zeros_like(occupancy, dtype=bool)
        seen_b = np.zeros_like(occupancy, dtype=bool)
        a = render_mid360(occupancy, origin, .2, all_allowed, position, seen_a)
        b = render_mid360(occupancy, origin, .2, none_allowed, position, seen_b)
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(seen_a, seen_b)
        self.assertTrue(seen_a.any())
        # The real wall remains visible even outside flight permission.
        self.assertTrue(np.any(np.abs(b[:, 0] - 3.1) < .11))


if __name__ == "__main__":
    unittest.main()
