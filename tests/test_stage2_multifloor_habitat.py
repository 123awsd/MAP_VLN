from __future__ import annotations

import unittest

from scripts.run_stage2_multifloor_habitat import interpolate_xyz


class MultifloorHabitatTest(unittest.TestCase):
    def test_interpolation_preserves_continuous_xyz(self):
        points = interpolate_xyz([[0, 0, 1], [0, 0, 2], [1, 0, 3]], .2)
        self.assertEqual(points[0], [0, 0, 1])
        self.assertEqual(points[-1], [1.0, 0.0, 3.0])
        self.assertTrue(all(
            max(abs(b[axis] - a[axis]) for axis in range(3)) <= .2 + 1e-9
            for a, b in zip(points, points[1:])
        ))
        self.assertGreater(len({round(point[2], 2) for point in points}), 3)


if __name__ == "__main__":
    unittest.main()
