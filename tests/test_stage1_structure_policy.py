import unittest

from stage1.structure_policy import normalize_structure_policy


class StructurePolicyTests(unittest.TestCase):
    def setUp(self):
        self.boxes = [{
            "instance_id": "boxer_0", "label": "bed",
            "center_xyz_m": [0.0, 0.0, 0.5], "size_xyz_m": [2.0, 1.8, 1.0],
            "detection_confidence": 0.9,
        }, {
            "instance_id": "boxer_1", "label": "door",
            "center_xyz_m": [2.0, 0.0, 1.0], "size_xyz_m": [0.9, 0.1, 2.0],
            "detection_confidence": 0.8,
        }]

    def test_only_high_confidence_furniture_is_removed(self):
        policy = normalize_structure_policy({"instances": [
            {"instance_id": "boxer_0", "role": "interior_object", "confidence": 0.95, "reason": "bed"},
            {"instance_id": "boxer_1", "role": "opening_boundary", "confidence": 0.99, "reason": "door"},
        ]}, self.boxes)
        self.assertTrue(policy["instances"][0]["remove_from_structure_map"])
        self.assertFalse(policy["instances"][1]["remove_from_structure_map"])
        self.assertTrue(all(item["keep_in_navigation_map"] for item in policy["instances"]))

    def test_omitted_instance_is_retained_fail_safe(self):
        policy = normalize_structure_policy({"instances": [
            {"instance_id": "boxer_0", "role": "interior_object", "confidence": 0.9, "reason": "bed"},
        ]}, self.boxes)
        self.assertEqual(policy["instances"][1]["role"], "uncertain")
        self.assertFalse(policy["instances"][1]["remove_from_structure_map"])

    def test_low_confidence_furniture_is_retained(self):
        policy = normalize_structure_policy({"instances": [
            {"instance_id": "boxer_0", "role": "interior_object", "confidence": 0.5, "reason": "unclear"},
            {"instance_id": "boxer_1", "role": "opening_boundary", "confidence": 0.9, "reason": "door"},
        ]}, self.boxes)
        self.assertFalse(policy["instances"][0]["remove_from_structure_map"])


if __name__ == "__main__":
    unittest.main()
