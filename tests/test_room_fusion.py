import unittest

from scripts.fuse_rooms_boxes import attach_small_fragments, canonicalize_rooms, classify_region


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

    def test_canonical_room_aggregates_fragment_geometry_objects_and_edges(self):
        rooms = [
            {
                "id": 1, "area_m2": 4.0, "centroid_xy_m": [0.0, 0.0],
                "polygon_xy_m": [[-1, -1], [1, -1], [1, 1], [-1, 1]],
                "adjacent_room_ids": [2, 3], "space_role": "room",
                "semantic_type": "bedroom", "semantic_score": 1.0,
                "objects": [{"id": "bed", "label": "bed"}],
            },
            {
                "id": 2, "parent_room_id": 1, "area_m2": 0.5,
                "centroid_xy_m": [1.5, 0.0],
                "polygon_xy_m": [[1.2, -0.5], [1.8, -0.5], [1.8, 0.5], [1.2, 0.5]],
                "adjacent_room_ids": [1, 3], "space_role": "room_fragment",
                "semantic_type": "bedroom", "semantic_score": 1.0,
                "objects": [{"id": "shelf", "label": "shelf"}],
            },
            {
                "id": 3, "area_m2": 1.2, "centroid_xy_m": [3.0, 0.0],
                "polygon_xy_m": [[2.5, -0.5], [3.5, -0.5], [3.5, 0.5], [2.5, 0.5]],
                "adjacent_room_ids": [1, 2], "space_role": "transition_space",
                "semantic_type": "corridor", "semantic_score": 1.0, "objects": [],
            },
        ]
        canonical, raw = canonicalize_rooms(rooms)
        self.assertEqual([room["id"] for room in canonical], [1, 3])
        room = canonical[0]
        self.assertEqual(room["merged_from_region_ids"], [1, 2])
        self.assertEqual({obj["id"] for obj in room["objects"]}, {"bed", "shelf"})
        self.assertEqual(room["adjacent_room_ids"], [3])
        self.assertEqual(len(room["polygon_components_xy_m"]), 2)
        self.assertGreater(len(room["polygon_xy_m"]), 3)
        self.assertEqual(raw[1]["object_ids"], ["shelf"])
        self.assertNotIn("objects", raw[1])


if __name__ == "__main__":
    unittest.main()
