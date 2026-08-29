import unittest

import numpy as np

from scripts.reconstruct_transition_spaces import (
    cluster_traversals,
    detect_transition_traversals,
    resample_polyline,
)
from scripts.run_transition_room_pipeline import annotate_regions, transition_masks


class TransitionReconstructionTests(unittest.TestCase):
    def test_detects_forward_and_reverse_adjacent_floor_traversals(self):
        dt = 0.1
        z = np.r_[
            np.full(30, 1.0), np.linspace(1.0, 4.0, 25), np.full(30, 4.0),
            np.linspace(4.0, 1.0, 25), np.full(30, 1.0),
        ]
        times = np.arange(len(z)) * dt
        trajectory = np.column_stack((np.linspace(0, 8, len(z)), np.zeros(len(z)), z))
        levels = [
            {"floor": 1, "floor_z_m": 0.0, "flight_z_m": 1.0},
            {"floor": 2, "floor_z_m": 3.0, "flight_z_m": 4.0},
        ]

        traversals = detect_transition_traversals(times, trajectory, levels)

        self.assertEqual([(item["source_floor"], item["target_floor"]) for item in traversals], [(1, 2), (2, 1)])
        self.assertTrue(all(len(item["centerline_xyz"]) >= 3 for item in traversals))

    def test_crossing_location_clusters_repeated_stair_usage(self):
        template = {
            "lower_floor": 1, "upper_floor": 2, "crossing_xyz": [1.0, 2.0, 2.5]
        }
        groups = cluster_traversals([
            template,
            {**template, "crossing_xyz": [1.3, 2.2, 2.5]},
            {**template, "crossing_xyz": [6.0, 2.0, 2.5]},
        ])
        self.assertEqual(sorted(map(len, groups)), [1, 2])

    def test_resampled_centerline_keeps_endpoints(self):
        points = np.asarray([[0, 0, 1], [0, 0, 1], [1, 0, 2], [2, 1, 3]], dtype=np.float32)
        result = resample_polyline(points, spacing=0.2)
        np.testing.assert_allclose(result[0], points[0])
        np.testing.assert_allclose(result[-1], points[-1])

    def test_transition_masks_and_postprocessing_preserve_ordinary_room(self):
        grid = np.zeros((100, 100), dtype=np.int8)
        metadata = {"origin_xy_m": [0.0, 0.0], "resolution_m": 0.1}
        sections = []
        for x in np.linspace(2.0, 5.0, 8):
            sections.append({
                "center_xyz": [x, 2.0, 2.0],
                "left_xyz": [x, 1.5, 2.0], "right_xyz": [x, 2.5, 2.0],
                "left_observed_wall": True, "right_observed_wall": True,
            })
        transition = {
            "id": "transition_1", "connected_floors": [1, 2],
            "classification": "stair_candidate", "confidence": 0.9,
            "boundary_closed": True, "lower_floor_opening_score": 0.8,
            "upper_floor_opening_score": 0.8, "boundary_sections": sections,
            "lower_entrance_xyz": [2.0, 2.0, 1.0],
            "upper_entrance_xyz": [5.0, 2.0, 4.0],
            "traversals": [{"centerline_xyz": [[2.0, 2.0, 1.0], [5.0, 2.0, 4.0]]}],
        }
        hard, soft, uncertain, ids, included = transition_masks(
            grid, metadata, [transition], 1
        )
        self.assertGreater(int(hard.sum()), 0)
        self.assertGreaterEqual(int(soft.sum()), int(hard.sum()))
        self.assertEqual(int(uncertain.sum()), 0)
        self.assertEqual(set(np.unique(ids)), {0, 1})

        raw = {"format": "test", "regions": [
            {"id": 1, "centroid_xy_m": [3.5, 2.0], "area_m2": 3.0,
             "polygon_xy_m": [[2.0, 1.4], [5.0, 1.4], [5.0, 2.6], [2.0, 2.6]]},
            {"id": 2, "centroid_xy_m": [8.0, 8.0], "area_m2": 4.0,
             "polygon_xy_m": [[7.0, 7.0], [9.0, 7.0], [9.0, 9.0], [7.0, 9.0]]},
        ]}
        post = annotate_regions(raw, grid, metadata, hard, soft, ids, included)
        roles = {region["id"]: region["space_role"] for region in post["regions"]}
        self.assertEqual(roles, {1: "transition_space", 2: "room"})


if __name__ == "__main__":
    unittest.main()
