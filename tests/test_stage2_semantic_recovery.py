import unittest

from stage2.semantic_recovery import recovery_inventory, validate_hypotheses


class SemanticRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.scene = {"rooms": [
            {"id": 1, "semantic_type": "living_room", "space_role": "room", "objects": [
                {"id": "tv1", "label": "television"}, {"id": "shelf1", "label": "shelf"},
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


if __name__ == "__main__":
    unittest.main()
