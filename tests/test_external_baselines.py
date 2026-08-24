from __future__ import annotations

import math
import unittest

from baselines.vln_metrics import dtw, evaluate_trajectory, path_length


class VLNMetricsTest(unittest.TestCase):
    def test_perfect_trajectory(self):
        path = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]
        result = evaluate_trajectory(path, path, success_distance=1.0)
        self.assertEqual(result["NE"], 0.0)
        self.assertTrue(result["SR"])
        self.assertTrue(result["OSR"])
        self.assertEqual(result["SPL"], 1.0)
        self.assertEqual(result["nDTW"], 1.0)

    def test_detour_reduces_spl(self):
        reference = [[0.0, 0.0], [2.0, 0.0]]
        trajectory = [[0.0, 0.0], [0.0, 2.0], [2.0, 0.0]]
        result = evaluate_trajectory(trajectory, reference, success_distance=1.0)
        self.assertTrue(result["SR"])
        self.assertLess(result["SPL"], 1.0)
        self.assertAlmostEqual(path_length(reference), 2.0)
        self.assertGreater(dtw(trajectory, reference), 0.0)

    def test_oracle_success_can_differ_from_final_success(self):
        reference = [[0.0, 0.0], [2.0, 0.0]]
        trajectory = [[0.0, 0.0], [2.0, 0.0], [5.0, 0.0]]
        result = evaluate_trajectory(trajectory, reference, success_distance=0.5)
        self.assertTrue(result["OSR"])
        self.assertFalse(result["SR"])
        self.assertEqual(result["SPL"], 0.0)
        self.assertTrue(math.isfinite(result["nDTW"]))


if __name__ == "__main__":
    unittest.main()
