from __future__ import annotations

import unittest

from stage2.vlm_verifier import normalize_verification


class VlmVerifierTest(unittest.TestCase):
    def test_normalizes_valid_result(self):
        result = normalize_verification({
            "found": True, "confidence": 0.92, "evidence": "A television is visible.",
            "visible_objects": ["Television", "cabinet", "television"],
            "target_bbox_0_1000": [100, 120, 700, 800],
        }, "television")
        self.assertTrue(result["found"])
        self.assertEqual(result["visible_objects"], ["cabinet", "television"])

    def test_rejects_non_boolean_found(self):
        with self.assertRaises(ValueError):
            normalize_verification({
                "found": "yes", "confidence": 0.9, "evidence": "visible",
                "visible_objects": [], "target_bbox_0_1000": None,
            }, "television")

    def test_rejects_invalid_bbox(self):
        with self.assertRaises(ValueError):
            normalize_verification({
                "found": True, "confidence": 0.9, "evidence": "visible",
                "visible_objects": [], "target_bbox_0_1000": [800, 0, 200, 500],
            }, "television")


if __name__ == "__main__":
    unittest.main()
