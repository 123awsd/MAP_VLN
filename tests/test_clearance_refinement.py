from __future__ import annotations

import unittest

import numpy as np

from stage2.clearance_refinement import refine_path_xy
from stage2.planning_contract import OCCUPANCY_SEMANTICS_HASH, MapIdentity, PlannerProfile
from stage2.voxel_map_3d import VoxelMap3D


def corridor_map() -> VoxelMap3D:
    profile = PlannerProfile(
        vehicle_radius_xy_m=.05, vehicle_radius_z_m=.05, safety_margin_m=.05,
        minimum_esdf_distance_m=.10, preferred_esdf_distance_m=.30,
        coarse_resolution_m=.10,
    )
    resolution = .10
    origin = np.asarray([-1.0, -1.0, 0.0])
    nz, ny, nx = 6, 20, 40
    y_centers = origin[1] + (np.arange(ny) + .5) * resolution
    free_y = np.abs(y_centers) < .40
    raw = np.full((nz, ny, nx), 100, dtype=np.int8)
    raw[:, free_y, :] = 0
    clearance_y = np.maximum(0.0, .40 - np.abs(y_centers))
    esdf = np.broadcast_to(clearance_y[None, :, None], raw.shape).astype(np.float32).copy()
    esdf[raw == 100] = 0.0
    identity = MapIdentity(
        "corridor-test", 1, 1, "content", OCCUPANCY_SEMANTICS_HASH, profile.profile_hash,
    )
    return VoxelMap3D(raw, esdf, origin, resolution, profile, identity)


class ClearanceRefinementTest(unittest.TestCase):
    def test_pushes_path_toward_corridor_center_and_preserves_z_endpoints(self):
        voxel_map = corridor_map()
        points = [[x, -.20, .25] for x in np.arange(0.0, 1.01, .10)]
        refined, report = refine_path_xy(points, voxel_map)

        self.assertEqual(refined[0], points[0])
        self.assertEqual(refined[-1], points[-1])
        self.assertTrue(all(a[2] == b[2] for a, b in zip(points, refined)))
        self.assertLess(np.mean(np.abs(np.asarray(refined)[1:-1, 1])), .20)
        self.assertLessEqual(report["max_lateral_shift_m"], .20 + 1e-9)
        self.assertTrue(all(voxel_map.line_is_valid(a, b) for a, b in zip(refined, refined[1:])))

    def test_path_without_horizontal_esdf_gradient_is_unchanged(self):
        voxel_map = corridor_map()
        points = [[0.0, 0.0, .25], [.5, 0.0, .25], [1.0, 0.0, .25]]
        refined, report = refine_path_xy(points, voxel_map)
        self.assertEqual(refined, points)
        self.assertFalse(report["accepted"])


if __name__ == "__main__":
    unittest.main()
