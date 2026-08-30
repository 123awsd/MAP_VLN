import unittest

import numpy as np

from stage2.grid_map import OccupancyGrid
from stage2.semantic_region_search import (
    SemanticRegionBelief,
    infer_region_type,
    materialize_semantic_regions,
    materialize_room_frontier_fallback,
    sample_region_points,
)


class SemanticRegionSearchTest(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid(np.zeros((30, 30), dtype=np.int8), [0, 0], 0.5, inflation_m=0.0)
        self.table = {
            "id": "table1", "label": "table", "center_xyz_m": [6.0, 6.0, 0.5],
            "size_xyz_m": [2.0, 1.0, 1.0], "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
            "probability": 0.8,
        }
        self.scene = {"rooms": [{
            "id": 1, "semantic_type": "kitchen", "space_role": "room",
            "centroid_xy_m": [6.0, 6.0],
            "polygon_xy_m": [[3.0, 3.0], [9.0, 3.0], [9.0, 9.0], [3.0, 9.0]],
            "objects": [self.table],
        }]}
        self.task = {
            "id": "find_cup", "verification_label": "cup",
            "target": {"label": "table", "room": None, "reference": None},
            "spatial_constraints": {
                "relation": None, "distance_m": [1.0, 2.0], "height_m": None,
                "face_target": True, "visibility_required": True,
            },
        }

    def test_region_type_uses_target_affordance(self):
        self.assertEqual(infer_region_type("cup", "table", "on"), "support_surface")
        self.assertEqual(infer_region_type("ball", "bed", "under"), "under_furniture")
        self.assertEqual(infer_region_type("bucket", "cabinet", "near"), "floor_near_anchor")
        self.assertEqual(infer_region_type("toilet paper", "toilet paper", "near"), "fixed_instance")

    def test_support_surface_samples_only_target_top(self):
        points = sample_region_points(self.table, "support_surface")
        self.assertEqual(len(points), 9)
        self.assertTrue(all(abs(point[2] - 1.0) < 1e-6 for point in points))

    def test_materialization_adds_region_geometry_scores(self):
        plan = {"hypotheses": [{
            "id": "recovery_1", "room_id": 1, "anchor_object_id": "table1",
            "semantic_region": "support_surface", "relation": "on", "relevance": "high",
        }]}
        values = materialize_semantic_regions(self.grid, self.scene, self.task, plan)
        self.assertTrue(values)
        self.assertTrue(all(value["candidate_source"] == "vlm_semantic_region" for value in values))
        self.assertTrue(all(value["semantic_prior_score"] == 1.0 for value in values))
        self.assertTrue(all(value["visible_region_sample_ids"] for value in values))
        self.assertTrue(all(value["region_cover_cost_m"] >= 0.0 for value in values))

    def test_negative_observation_only_penalizes_new_coverage(self):
        candidate = {
            "semantic_region_id": "r1", "semantic_prior_score": 1.0,
            "visible_region_sample_ids": [0, 1], "region_sample_count": 4,
            "view_quality": 1.0, "base_geometry_cost": 0.0, "terminal_cost": 0.0,
        }
        belief = SemanticRegionBelief(detector_reliability=0.8)
        belief.register([candidate])
        first = belief.observe(candidate, found=False)
        belief.apply([candidate])
        second = belief.observe(candidate, found=False)
        self.assertAlmostEqual(first["new_belief"], 0.6)
        self.assertAlmostEqual(second["new_belief"], 0.6)
        self.assertGreater(candidate["terminal_cost"], 0.0)

    def test_room_frontier_is_explicit_low_priority_fallback(self):
        plan = {"hypotheses": [{"room_id": 1, "anchor_object_id": "table1"}]}
        values = materialize_room_frontier_fallback(self.grid, self.scene, self.task, plan)
        self.assertTrue(values)
        self.assertTrue(all(value["candidate_source"] == "room_frontier_fallback" for value in values))
        self.assertTrue(all(value["semantic_prior_score"] == 0.15 for value in values))


if __name__ == "__main__":
    unittest.main()
