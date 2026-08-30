from __future__ import annotations

import unittest

from stage2.spatial_verification import effective_relation_context, verify_target_reference_relation


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

    def test_historical_recovery_does_not_inherit_original_anchor_relation(self):
        task = {
            "target": {"label": "vanity", "references": []},
            "verification_label": "toilet paper",
            "spatial_constraints": {"relation": "near"},
        }
        candidate = {
            "object_id": "paper", "recovery_search": {
                "source": "historical_target_box", "anchor_object_id": "paper", "relation": "near",
            },
        }
        context = effective_relation_context(task, candidate, {"paper": self.obj("paper", [1, 2, 1], [.2, .2, .2])})
        self.assertIsNone(context["relation"])
        self.assertEqual(context["scope"], "historical_target_box_visual_recheck")
        self.assertTrue(verify_target_reference_relation(context["relation"], context["target"], context["references"])["valid"])

    def test_recovery_anchor_uses_detected_target_geometry(self):
        task = {
            "target": {"label": "vanity", "references": []},
            "verification_label": "toilet paper",
            "spatial_constraints": {"relation": "near"},
        }
        anchor = self.obj("sink", [1, 1, 1], [1, 1, 1])
        candidate = {
            "object_id": "sink", "recovery_search": {
                "source": "semantic_anchor", "anchor_object_id": "sink", "relation": "near",
            },
        }
        detected = [{"confirmed": True, "score": .8, "center": [1.2, 1, 1], "size": [.2, .2, .2]}]
        context = effective_relation_context(task, candidate, {"sink": anchor}, detected)
        self.assertEqual(context["references"][0]["id"], "sink")
        self.assertTrue(verify_target_reference_relation(context["relation"], context["target"], context["references"])["valid"])


if __name__ == "__main__":
    unittest.main()
