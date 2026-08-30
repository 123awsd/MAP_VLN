import unittest

from stage2.semantic_recovery import (
    merge_historical_target_boxes,
    recovery_inventory,
    validate_hypotheses,
)


class SemanticRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.scene = {"rooms": [
            {"id": 1, "semantic_type": "living_room", "space_role": "room", "objects": [
                {"id": "tv1", "label": "television", "probability": 0.82}, {"id": "shelf1", "label": "shelf"},
            ]},
            {"id": 2, "semantic_type": "corridor", "space_role": "transition_space", "objects": [
                {"id": "door1", "label": "door"},
            ]},
            {"id": 3, "semantic_type": "living_room", "space_role": "room_fragment", "objects": [
                {"id": "duplicate_tv", "label": "television"},
            ]},
        ]}

    def test_inventory_excludes_transition_space(self):
        inventory = recovery_inventory(self.scene)
        self.assertEqual([room["room_id"] for room in inventory["rooms"]], [1])

    def test_validation_rejects_hallucinated_and_failed_anchors(self):
        raw = {"target_label": "television", "hypotheses": [
            {"room_id": 1, "anchor_object_id": "tv1", "relevance": "high"},
            {"room_id": 1, "anchor_object_id": "made_up", "relevance": "high"},
            {"room_id": 1, "anchor_object_id": "shelf1", "relevance": "medium"},
        ]}
        result = validate_hypotheses(raw, self.scene, {"tv1"}, 3)
        self.assertEqual([item["anchor_object_id"] for item in result["hypotheses"]], ["shelf1"])
        self.assertEqual(result["hypotheses"][0]["semantic_region"], "support_surface")

    def test_invalid_region_name_falls_back_to_geometry_family(self):
        raw = {"target_label": "television", "hypotheses": [{
            "room_id": 1, "anchor_object_id": "tv1", "relation": "near",
            "relevance": "high", "semantic_region": "magic_corner",
        }]}
        result = validate_hypotheses(raw, self.scene, set(), 3)
        self.assertEqual(result["hypotheses"][0]["semantic_region"], "instance_region")

    def test_legacy_cached_region_name_is_canonicalized(self):
        raw = {"target_label": "ball", "hypotheses": [{
            "room_id": 1, "anchor_object_id": "shelf1", "relation": "under",
            "relevance": "high", "semantic_region": "under_furniture",
        }]}
        result = validate_hypotheses(raw, self.scene, set(), 3)
        self.assertEqual(result["hypotheses"][0]["semantic_region"], "below_region")

    def test_historical_target_box_is_annotated_for_joint_ranking(self):
        raw = {"target_label": "television", "hypotheses": [{
            "room_id": 1, "anchor_object_id": "tv1", "relevance": "high",
        }]}
        item = validate_hypotheses(raw, self.scene, set(), 3)["hypotheses"][0]
        self.assertEqual(item["source"], "historical_target_box")
        self.assertEqual(item["historical_confidence"], 0.82)

    def test_exact_historical_box_is_preserved_alongside_qwen_anchors(self):
        inferred = {"target_label": "television", "hypotheses": [{
            "id": "recovery_1", "room_id": 1, "anchor_object_id": "shelf1",
            "anchor_label": "shelf", "relevance": "high",
            "historical_confidence": 0.5, "source": "semantic_anchor",
        }]}
        merged = merge_historical_target_boxes(inferred, self.scene, set(), 3)
        ids = [item["anchor_object_id"] for item in merged["hypotheses"]]
        self.assertIn("tv1", ids)
        self.assertIn("shelf1", ids)


if __name__ == "__main__":
    unittest.main()
