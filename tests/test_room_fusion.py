import unittest

from scripts.fuse_rooms_boxes import attach_small_fragments, classify_region


class RoomFusionTests(unittest.TestCase):
    def test_small_connector_is_corridor(self):
        room = {
            "area_m2": 1.4,
            "adjacent_room_ids": [2, 3],
            "objects": [{"label": "cabinet", "probability": 0.9}],
        }
        self.assertEqual(classify_region(room), ("corridor", 1.0, "transition_space"))

    def test_large_destination_keeps_semantic_room_type(self):
        room = {
            "area_m2": 5.0,
            "adjacent_room_ids": [2],
            "objects": [{"label": "bed", "probability": 0.8}],
        }
        self.assertEqual(classify_region(room), ("bedroom", 1.6, "room"))

    def test_small_fragment_is_attached_without_deleting_geometry(self):
        rooms = [
            {"id": 1, "area_m2": 5.0, "centroid_xy_m": [0, 0], "space_role": "room",
             "semantic_type": "bedroom", "semantic_score": 0.8},
            {"id": 2, "area_m2": 0.5, "centroid_xy_m": [1, 0], "space_role": "room",
             "semantic_type": "unknown", "semantic_score": 0.0},
        ]
        attach_small_fragments(rooms)
        self.assertEqual(rooms[1]["space_role"], "room_fragment")
        self.assertEqual(rooms[1]["parent_room_id"], 1)
        self.assertEqual(rooms[1]["semantic_type"], "bedroom")


if __name__ == "__main__":
    unittest.main()
