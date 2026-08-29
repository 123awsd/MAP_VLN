from __future__ import annotations

import unittest

import numpy as np

from stage2.bspline_3d import BsplineSettings, anchor_astar_path, plan_collision_checked_bspline
from stage2.planning_contract import OCCUPANCY_SEMANTICS_HASH, MapIdentity, PlannerProfile
from stage2.voxel_map_3d import VoxelMap3D


def free_map() -> VoxelMap3D:
    profile = PlannerProfile(
        vehicle_radius_xy_m=0, vehicle_radius_z_m=0, safety_margin_m=0,
        minimum_esdf_distance_m=.1, preferred_esdf_distance_m=.2,
        coarse_resolution_m=.2,
    )
    raw = np.zeros((20, 20, 20), dtype=np.int8)
    esdf = np.ones_like(raw, dtype=np.float32)
    identity = MapIdentity(
        "test", 1, 1, "content", OCCUPANCY_SEMANTICS_HASH, profile.profile_hash,
    )
    return VoxelMap3D(raw, esdf, [0, 0, 0], .1, profile, identity)


class Bspline3DTest(unittest.TestCase):
    def test_full_bspline_is_continuous_and_collision_checked(self):
        voxel_map = free_map()
        result = plan_collision_checked_bspline(
            [[.2, .2, .2], [.8, .2, .5], [1.4, .7, .8], [1.7, 1.2, 1.0]],
            voxel_map,
            BsplineSettings(sample_step_m=.05),
        )
        self.assertEqual(result.mode, "full_bspline")
        self.assertEqual(result.degree, 3)
        self.assertGreater(len(result.points_xyz_m), 20)
        self.assertTrue(all(voxel_map.is_state_valid(point) for point in result.points_xyz_m))

    def test_prefix_split_preserves_endpoints(self):
        voxel_map = free_map()
        points = [[.2, .2, .2], [.8, .2, .2], [1.4, .2, .2], [1.8, .2, .2]]
        result = plan_collision_checked_bspline(
            points, voxel_map,
            BsplineSettings(sample_step_m=.05, conditional_prefix_length_m=.5),
        )
        self.assertEqual(result.points_xyz_m[0], points[0])
        self.assertEqual(result.points_xyz_m[-1], points[-1])

    def test_actual_endpoints_are_connected_to_snapped_path(self):
        voxel_map = free_map()
        anchored = anchor_astar_path(
            [[.3, .3, .3], [.9, .3, .3]],
            [.21, .21, .21], [1.01, .31, .31], voxel_map,
        )
        self.assertEqual(anchored[0], [.21, .21, .21])
        self.assertEqual(anchored[-1], [1.01, .31, .31])


if __name__ == "__main__":
    unittest.main()
