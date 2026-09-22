import math
import unittest

import numpy as np

from stage2.narrow_corridor_geometry import (
    NarrowCorridorInfeasible,
    centered_tube_half_width,
    detect_two_sided_bottleneck,
    robust_local_tangent,
    unsigned_direction_error_deg,
)


def wall_cloud(origin, tangent, half_width=0.28, length=0.8, count=21):
    origin = np.asarray(origin, dtype=float)
    tangent = np.asarray(tangent, dtype=float)
    tangent /= np.linalg.norm(tangent)
    normal = np.asarray([-tangent[1], tangent[0]])
    values = []
    for along in np.linspace(-length, length, count):
        for side in (-1.0, 1.0):
            xy = origin[:2] + along * tangent + side * half_width * normal
            values.append([xy[0], xy[1], origin[2]])
    return np.asarray(values)


class NarrowCorridorGeometryTests(unittest.TestCase):
    def test_diagonal_doorway_is_detected_in_local_frame(self):
        tangent = np.asarray([math.cos(math.radians(35)), math.sin(math.radians(35))])
        point = np.asarray([1.0, -0.5, 0.45])
        result = detect_two_sided_bottleneck(point, tangent, wall_cloud(point, tangent))
        self.assertIsNotNone(result)
        self.assertLess(result["width_m"], 0.7)
        self.assertLess(unsigned_direction_error_deg(result["tangent"], tangent), 1e-6)

    def test_single_wall_is_not_called_a_bottleneck(self):
        tangent = np.asarray([1.0, 0.0])
        point = np.asarray([0.0, 0.0, 0.45])
        cloud = wall_cloud(point, tangent)[::2]
        self.assertIsNone(detect_two_sided_bottleneck(point, tangent, cloud))

    def test_curved_narrow_path_keeps_local_directions(self):
        angles = np.linspace(0.0, math.pi / 2.0, 9)
        points = np.asarray([[math.cos(a), math.sin(a), 0.45] for a in angles])
        tangents = [robust_local_tangent(points, i, minimum_span_m=0.05) for i in range(1, len(points) - 1)]
        self.assertGreater(unsigned_direction_error_deg(tangents[0], tangents[-1]), 60.0)
        self.assertLess(unsigned_direction_error_deg(tangents[3], [-1.0, 1.0]), 10.0)

    def test_infeasible_centered_tube_fails_explicitly(self):
        with self.assertRaises(NarrowCorridorInfeasible):
            centered_tube_half_width(
                [0.0, 0.4, 0.45], [1.0, 0.4, 0.45], [0.0, 0.0, 0.45], [0.0, 1.0, 0.0],
                available_half_width_m=0.10, maximum_half_width_m=0.18,
            )


if __name__ == "__main__":
    unittest.main()
