from __future__ import annotations

import unittest

from stage2.spatial_verification import verify_target_reference_relation


class SpatialVerificationTest(unittest.TestCase):
    @staticmethod
    def obj(identifier, center, size):
        return {"id": identifier, "center_xyz_m": center, "size_xyz_m": size}

    def test_on_accepts_supported_target(self):
        target = self.obj("microwave", [0, 0, 1.1], [0.5, 0.4, 0.4])
        counter = self.obj("counter", [0, 0, 0.6], [1.5, 0.7, 0.6])
        result = verify_target_reference_relation("on", target, [counter])
        self.assertTrue(result["valid"])

    def test_near_rejects_distant_reference(self):
        target = self.obj("lamp", [0, 0, 1], [0.2, 0.2, 0.5])
        bed = self.obj("bed", [4, 0, 0.5], [2, 1, 0.5])
        self.assertFalse(verify_target_reference_relation("near", target, [bed])["valid"])


if __name__ == "__main__":
    unittest.main()
