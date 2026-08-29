from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from stage2.astar_3d import CoarseAstar3D
from stage2.joint_planner import plan_joint_mission
from stage2.motion_cost_oracle import MotionCostOracle
from stage2.planning_contract import OCCUPANCY_SEMANTICS_HASH, MapIdentity, PlannerProfile
from stage2.voxel_map_3d import VoxelMap3D


def oracle(version=1, cache=None):
    profile = PlannerProfile(
        vehicle_radius_xy_m=0, vehicle_radius_z_m=0, safety_margin_m=0,
        minimum_esdf_distance_m=0, preferred_esdf_distance_m=.7,
        coarse_resolution_m=.2, nearest_free_radius_m=.2,
    )
    raw = np.zeros((20, 10, 20), dtype=np.int8)
    esdf = np.ones_like(raw, dtype=np.float32)
    identity = MapIdentity("epoch", version, 1, f"content-{version}", OCCUPANCY_SEMANTICS_HASH, profile.profile_hash)
    return MotionCostOracle(CoarseAstar3D(VoxelMap3D(raw, esdf, [0, 0, 0], .1, profile, identity)), cache)


class MotionCostOracleTest(unittest.TestCase):
    def test_cache_is_bound_to_geometry_version(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "cache.json"
            first = oracle(1, path)
            self.assertIsNotNone(first.path([.3, .3, .3], [1.5, .3, .3]))
            self.assertEqual(first.cache_misses, 1)
            same = oracle(1, path)
            self.assertIsNotNone(same.path([.3, .3, .3], [1.5, .3, .3]))
            self.assertEqual(same.cache_hits, 1)
            changed = oracle(2, path)
            self.assertIsNotNone(changed.path([.3, .3, .3], [1.5, .3, .3]))
            self.assertEqual(changed.cache_misses, 1)

    def test_joint_planner_uses_xyz_path(self):
        motion = oracle()
        tasks = {"tasks": [
            {"id": "upper", "active_initially": True, "prerequisites": []},
            {"id": "near", "active_initially": True, "prerequisites": []},
        ]}
        def candidate(task, x, z):
            return {"id": task, "object_id": task, "pose": {"x": x, "y": .3, "z": z, "yaw": 0}, "terminal_cost": 0}
        result = plan_joint_mission(
            motion, tasks,
            {"upper": [candidate("upper", 1.5, 1.5)], "near": [candidate("near", .7, .3)]},
            [.3, .3, .3, 0],
        )
        self.assertEqual(result["planner"], "subset_dp_astar_3d")
        self.assertEqual(result["visits"][0]["task_id"], "near")
        self.assertIn("points_xyz_m", result["segments"][0])
        self.assertEqual(result["start_xyz_yaw"], [.3, .3, .3, 0])
        self.assertEqual(result["map_epoch_uuid"], "epoch")
        self.assertEqual(
            result["planner_profile_hash"],
            motion.identity.planner_profile_hash,
        )


if __name__ == "__main__":
    unittest.main()
