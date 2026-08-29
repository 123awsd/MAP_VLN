from __future__ import annotations

import unittest

import numpy as np

from stage2.planning_contract import (
    OCCUPANCY_SEMANTICS_HASH,
    MapIdentity,
    OccupancyState,
    PlannerProfile,
)
from stage2.voxel_map_3d import VoxelMap3D


def identity(profile):
    return MapIdentity("test-map", 1, 1, "content", OCCUPANCY_SEMANTICS_HASH, profile.profile_hash)


class VoxelMap3DTest(unittest.TestCase):
    def test_unknown_stays_blocked_even_with_large_esdf(self):
        profile = PlannerProfile(
            vehicle_radius_xy_m=0.05, vehicle_radius_z_m=0.05, safety_margin_m=0.05,
            minimum_esdf_distance_m=0.2, preferred_esdf_distance_m=0.2,
            coarse_resolution_m=0.2,
        )
        raw = np.full((4, 4, 4), int(OccupancyState.UNKNOWN), dtype=np.int8)
        raw[1, 1, 1] = int(OccupancyState.FREE)
        esdf = np.full(raw.shape, 10.0, dtype=np.float32)
        grid = VoxelMap3D(raw, esdf, [0, 0, 0], 0.1, profile, identity(profile))
        self.assertTrue(grid.is_state_valid([0.15, 0.15, 0.15]))
        self.assertFalse(grid.is_state_valid([0.25, 0.15, 0.15]))

    def test_esdf_threshold_defines_inflated_free(self):
        profile = PlannerProfile(
            vehicle_radius_xy_m=0.1, vehicle_radius_z_m=0.1, safety_margin_m=0.1,
            minimum_esdf_distance_m=0.3, preferred_esdf_distance_m=0.3,
            coarse_resolution_m=0.2,
        )
        raw = np.zeros((2, 2, 2), dtype=np.int8)
        esdf = np.full(raw.shape, 0.29, dtype=np.float32)
        esdf[0, 0, 0] = 0.30
        grid = VoxelMap3D(raw, esdf, [0, 0, 0], 0.1, profile, identity(profile))
        self.assertTrue(grid.is_state_valid([0.05, 0.05, 0.05]))
        self.assertFalse(grid.is_state_valid([0.15, 0.05, 0.05]))

    def test_coarse_cell_requires_every_fine_voxel_free(self):
        profile = PlannerProfile(
            vehicle_radius_xy_m=0.04, vehicle_radius_z_m=0.04, safety_margin_m=0.04,
            minimum_esdf_distance_m=0.1, preferred_esdf_distance_m=0.1,
            coarse_resolution_m=0.2,
        )
        raw = np.zeros((4, 4, 4), dtype=np.int8)
        esdf = np.ones(raw.shape, dtype=np.float32)
        raw[0, 0, 0] = int(OccupancyState.UNKNOWN)
        grid = VoxelMap3D(raw, esdf, [0, 0, 0], 0.1, profile, identity(profile))
        coarse, resolution = grid.conservative_coarse_free()
        self.assertEqual(coarse.shape, (2, 2, 2))
        self.assertFalse(coarse[0, 0, 0])
        self.assertTrue(coarse[1, 1, 1])
        self.assertAlmostEqual(resolution, 0.2)


if __name__ == "__main__":
    unittest.main()
