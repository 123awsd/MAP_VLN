import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from live_clearance_core import assess, cloud_xyz


class GeometryTests(unittest.TestCase):
    def test_door_center_and_lateral_error(self):
        x = np.linspace(-2, 2, 401)
        points = np.vstack([np.column_stack([x, x * 0 + y, x * 0])
                            for y in (-0.3, 0.3)])
        self.assertEqual(assess(points, [0, 0, 0], [0.2, 0, 0])["status"],
                         "NO_HIT_IN_RECEIVED_POINTS")
        self.assertEqual(assess(points, [0, 0.081, 0], [0.2, 0, 0])["status"], "WOULD_STOP")

    def test_forward_obstacle_detected_before_hard_distance(self):
        r = assess([[0.5, 0, 0]], [0, 0, 0], [0.6, 0, 0])
        self.assertEqual(r["reason"], "braking_capsule")

    def test_behind_and_thin_obstacle(self):
        self.assertEqual(assess([[-0.5, 0, 0]], [0, 0, 0], [0.6, 0, 0])["reason"], "none")
        self.assertEqual(assess([[0, 0, 0.21]], [0, 0, 0], [0, 0, 0])["reason"], "near_obstacle")

    def test_invalid_inputs(self):
        for points in ([], [[np.nan, 0, 0]]):
            with self.assertRaises(ValueError):
                assess(points, [0, 0, 0], [0, 0, 0])

    def test_cloud_padded_big_endian(self):
        data = bytearray(32)
        np.ndarray((2, 3), dtype=">f4", buffer=data, strides=(16, 4))[:] = [[1, 2, 3], [4, 5, 6]]
        msg = NS(width=1, height=2, row_step=16, point_step=12, data=data,
                 is_bigendian=True, fields=[NS(name=n, offset=i*4, count=1, datatype=7)
                                           for i, n in enumerate("xyz")])
        np.testing.assert_equal(cloud_xyz(msg), [[1, 2, 3], [4, 5, 6]])
        msg.data = data[:12]
        with self.assertRaises(ValueError):
            cloud_xyz(msg)


if __name__ == "__main__":
    unittest.main()
