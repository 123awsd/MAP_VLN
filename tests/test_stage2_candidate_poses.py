from __future__ import annotations

import math
import unittest

import numpy as np

from stage2.candidate_poses import generate_all_candidates, generate_candidates
from stage2.grid_map import OccupancyGrid


class CandidatePoseTest(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid(
            np.zeros((60, 60), dtype=np.int8), [0.0, 0.0], 0.5, inflation_m=0.0
        )
        self.scene = {
            "rooms": [{
                "id": 1,
                "floor_id": 1,
                "semantic_type": "living_room",
                "objects": [
                    {
                        "id": "target",
                        "label": "television",
                        "center_xyz_m": [10.0, 10.0, 1.0],
                        "size_xyz_m": [0.6, 0.4, 0.8],
                        "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                        "probability": 1.0,
                    },
                    {
                        "id": "ref_a",
                        "label": "sofa",
                        "center_xyz_m": [8.0, 10.0, 0.5],
                        "size_xyz_m": [0.8, 0.8, 0.8],
                        "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                        "probability": 1.0,
                    },
                    {
                        "id": "ref_b",
                        "label": "table",
                        "center_xyz_m": [12.0, 10.0, 0.5],
                        "size_xyz_m": [0.8, 0.8, 0.8],
                        "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                        "probability": 1.0,
                    },
                ],
            }, {
                "id": 2,
                "floor_id": 2,
                "semantic_type": "living_room",
                "objects": [{
                    "id": "wrong_floor_ref", "label": "sofa",
                    "center_xyz_m": [10.0, 10.0, 4.0],
                    "size_xyz_m": [1.0, 1.0, 1.0],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "probability": 2.0,
                }],
            }]
        }

    @staticmethod
    def task(label="television", relation=None, constraints=None, **target):
        spatial_constraints = {
            "relation": relation,
            "distance_m": [1.0, 2.0],
            "height_m": None,
            "height_range_m": None,
            "region_type": "auto",
            "observation_detail": "normal",
            "vertical_fov_deg": 70.0,
            "horizontal_fov_deg": 90.0,
            "yaw_tolerance_deg": 55.0,
            "face_target": True,
            "visibility_required": True,
        }
        spatial_constraints.update(constraints or {})
        return {
            "id": "inspect_target",
            "target": {"label": label, "room": None, "reference": None, **target},
            "spatial_constraints": spatial_constraints,
        }

    def test_directional_relation_is_hard_filtered(self):
        candidates = generate_candidates(self.grid, self.scene, self.task(relation="front"), max_candidates=8)
        self.assertTrue(candidates)
        self.assertTrue(all(
            abs(math.atan2(math.sin(item["candidate_angle_rad"]), math.cos(item["candidate_angle_rad"])))
            <= math.radians(55.0)
            for item in candidates
        ))
        self.assertTrue(all(item["line_of_sight"] for item in candidates))

    def test_facing_only_constrains_terminal_heading(self):
        candidates = generate_candidates(self.grid, self.scene, self.task(relation="facing"), max_candidates=24)
        self.assertGreaterEqual(len({round(item["candidate_angle_rad"], 2) for item in candidates}), 4)
        for item in candidates:
            pose = item["pose"]
            expected_yaw = math.atan2(10.0 - pose["y"], 10.0 - pose["x"])
            self.assertAlmostEqual(pose["yaw"], expected_yaw)

    def test_support_surface_uses_region_and_height(self):
        candidates = generate_candidates(
            self.grid,
            self.scene,
            self.task(label="table", relation="on"),
            max_candidates=8,
        )
        self.assertTrue(candidates)
        self.assertTrue(all(item["region_type"] == "support_surface" for item in candidates))
        self.assertTrue(all(item["pose"]["z"] > 0.5 for item in candidates))
        self.assertTrue(all(item["view_quality"] > 0.0 for item in candidates))

    def test_target_reference_relation_keeps_view_around_target_and_scopes_reference(self):
        candidates = generate_candidates(
            self.grid, self.scene,
            self.task(relation="near", reference="sofa"),
            max_candidates=8,
        )
        self.assertTrue(candidates)
        self.assertTrue(all(item["region_type"] == "instance_region" for item in candidates))
        self.assertTrue(all(item["reference_object_ids"] == ["ref_a"] for item in candidates))

    def test_between_relation_requires_two_references_in_geometry(self):
        candidates = generate_candidates(
            self.grid,
            self.scene,
            self.task(relation="between", references=["sofa", "table"]),
            max_candidates=8,
        )
        self.assertTrue(candidates)
        self.assertTrue(all(8.0 <= item["pose"]["x"] <= 12.0 for item in candidates))
        self.assertTrue(all(item["between_distance_m"] <= 1.0 for item in candidates))

    def test_fine_observation_keeps_distinct_height_candidates(self):
        task = self.task(constraints={
            "observation_detail": "fine",
            "height_range_m": [0.8, 1.6],
            "vertical_fov_deg": 120.0,
        })
        candidates = generate_candidates(self.grid, self.scene, task, max_candidates=20)
        self.assertGreaterEqual(len({round(item["pose"]["z"], 3) for item in candidates}), 3)

    def test_region_and_detail_change_preferred_observation_distance(self):
        normal = generate_candidates(
            self.grid, self.scene,
            self.task(constraints={"region_type": "surrounding_region"}),
            max_candidates=1,
        )[0]
        fine = generate_candidates(
            self.grid, self.scene,
            self.task(constraints={"region_type": "surrounding_region", "observation_detail": "fine"}),
            max_candidates=1,
        )[0]
        self.assertGreater(normal["preferred_observation_distance_m"], fine["preferred_observation_distance_m"])
        self.assertEqual(normal["distance_to_box_surface_m"], normal["preferred_observation_distance_m"])
        self.assertEqual(fine["distance_to_box_surface_m"], fine["preferred_observation_distance_m"])


if __name__ == "__main__":
    unittest.main()
