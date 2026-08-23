from __future__ import annotations

import unittest

from stage2.semantic_fusion import OnlineSemanticFusion


def detection(label="cup", center=(1.0, 2.0, 1.0), associated=None, confirmed=False):
    return {
        "label": label, "center": list(center), "size": [0.2, 0.2, 0.3], "score": 0.6,
        "associated_object_id": associated, "confirmed": confirmed,
    }


class SemanticFusionTest(unittest.TestCase):
    def test_novel_object_requires_two_views(self):
        fusion = OnlineSemanticFusion()
        self.assertEqual(fusion.update([detection()], 1), [])
        tracks = fusion.update([detection(center=(1.1, 2.0, 1.0))], 2)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]["source"], "online_novel")
        self.assertEqual(tracks[0]["support_count"], 2)

    def test_far_detections_make_separate_unconfirmed_tracks(self):
        fusion = OnlineSemanticFusion(association_radius_m=0.5)
        fusion.update([detection(center=(0, 0, 1))], 1)
        fusion.update([detection(center=(2, 0, 1))], 2)
        self.assertEqual(fusion.snapshot(), [])
        self.assertEqual(len(fusion.snapshot(confirmed_only=False)), 2)

    def test_known_object_is_immediately_confirmed(self):
        fusion = OnlineSemanticFusion()
        tracks = fusion.update([detection(associated="box_7", confirmed=True)], 1)
        self.assertEqual(tracks[0]["associated_object_id"], "box_7")

    def test_materializes_confirmed_novel_object(self):
        fusion = OnlineSemanticFusion()
        fusion.update([detection()], 1)
        fusion.update([detection(center=(1.05, 2.0, 1.0))], 2)
        graph = {"rooms": [{"id": "room_0", "centroid_xy_m": [1.0, 2.0], "objects": []}]}
        updated, added = fusion.materialize_scene_graph(graph)
        self.assertEqual(len(added), 1)
        self.assertEqual(updated["rooms"][0]["objects"][0]["source"], "online_owlv2_rgbd_fusion")

    def test_configurable_support_threshold_is_respected(self):
        fusion = OnlineSemanticFusion(minimum_novel_support=3)
        fusion.update([detection()], 1)
        fusion.update([detection(center=(1.05, 2.0, 1.0))], 2)
        self.assertEqual(fusion.snapshot(), [])
        self.assertFalse(fusion.snapshot(confirmed_only=False)[0]["confirmed"])
        self.assertEqual(len(fusion.update([detection(center=(1.1, 2.0, 1.0))], 3)), 1)


if __name__ == "__main__":
    unittest.main()
