from __future__ import annotations

import unittest

import numpy as np

from stage2.astar_3d import AstarFailure, CoarseAstar3D
from stage2.planning_contract import (
    OCCUPANCY_SEMANTICS_HASH,
    MapIdentity,
    OccupancyState,
    PlannerProfile,
)
from stage2.voxel_map_3d import VoxelMap3D


def make_map(raw, profile):
    esdf = np.where(raw == int(OccupancyState.FREE), 10.0, 0.0).astype(np.float32)
    identity = MapIdentity("test", 1, 1, "content", OCCUPANCY_SEMANTICS_HASH, profile.profile_hash)
    return VoxelMap3D(raw, esdf, [0, 0, 0], 0.1, profile, identity)


class Astar3DTest(unittest.TestCase):
    def setUp(self):
        self.profile = PlannerProfile(
            vehicle_radius_xy_m=0.04,
            vehicle_radius_z_m=0.04,
            safety_margin_m=0.04,
            minimum_esdf_distance_m=0.1,
            preferred_esdf_distance_m=0.1,
            coarse_resolution_m=0.2,
            nearest_free_radius_m=0.2,
        )

    def test_routes_in_xyz_and_changes_height(self):
        raw = np.zeros((12, 12, 12), dtype=np.int8)
        planner = CoarseAstar3D(make_map(raw, self.profile))
        path = planner.plan([0.3, 0.3, 0.3], [0.9, 0.9, 0.9])
        self.assertNotIsInstance(path, AstarFailure)
        self.assertGreater(path.vertical_distance_m, 0.5)
        self.assertTrue(all(len(point) == 3 for point in path.points_xyz_m))

    def test_unknown_slab_is_not_crossed(self):
        raw = np.zeros((12, 12, 12), dtype=np.int8)
        raw[:, :, 6:8] = int(OccupancyState.UNKNOWN)
        planner = CoarseAstar3D(make_map(raw, self.profile))
        result = planner.plan([0.3, 0.3, 0.3], [0.9, 0.3, 0.3])
        self.assertIsInstance(result, AstarFailure)
        self.assertEqual(result.status, "no_path")

    def test_diagonal_cannot_cut_occupied_corner(self):
        raw = np.full((4, 4, 4), int(OccupancyState.UNKNOWN), dtype=np.int8)
        raw[0:2, 0:2, 0:2] = int(OccupancyState.FREE)
        raw[0:2, 2:4, 2:4] = int(OccupancyState.FREE)
        planner = CoarseAstar3D(make_map(raw, self.profile))
        result = planner.plan([0.1, 0.1, 0.1], [0.3, 0.3, 0.1])
        self.assertIsInstance(result, AstarFailure)


if __name__ == "__main__":
    unittest.main()
