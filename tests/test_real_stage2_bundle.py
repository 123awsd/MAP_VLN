import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "real_fly/stage2_runtime/scripts/prepare_execution_bundle.py"


class RealStage2BundleTest(unittest.TestCase):
    def test_same_position_observation_is_not_dropped(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission_path = root / "mission.json"
            map_path = root / "map.pcd"
            metadata_path = root / "metadata.json"
            output_path = root / "bundle.json"
            identity = {
                "map_epoch_uuid": "epoch",
                "geometry_map_version": 1,
                "semantic_map_version": 1,
                "snapshot_content_hash": "snapshot",
                "occupancy_semantics_hash": "occupancy",
                "planner_profile_hash": "profile",
            }
            mission = {
                "format": "pre_map_vln.mission_plan.v1",
                "map_epoch_uuid": "epoch",
                "geometry_map_version": 1,
                "planner_profile_hash": "profile",
                "start_xyz_yaw": [1.0, 2.0, 0.8, 0.0],
                "real_start_source": "explicit_approved_pose",
                "start_pose_explicitly_approved": True,
                "visits": [{
                    "task_id": "inspect_target",
                    "candidate_id": "candidate_1",
                    "pose": {"x": 1.0, "y": 2.0, "z": 0.8, "yaw": 1.2},
                }],
                "segments": [{
                    "points_xyz_m": [[1.0, 2.0, 0.8]],
                    "validated_trajectory": {
                        "execution_waypoints_xyz_m": [[1.0, 2.0, 0.8]],
                    },
                }],
            }
            mission_path.write_text(json.dumps(mission), encoding="utf-8")
            metadata_path.write_text(json.dumps({"frame_id": "world", "identity": identity}), encoding="utf-8")
            map_path.write_bytes(b"pcd-test")

            subprocess.run([
                sys.executable, str(SCRIPT), "--mission", str(mission_path),
                "--map", str(map_path), "--voxel-metadata", str(metadata_path),
                "--output", str(output_path),
            ], check=True, capture_output=True, text=True)
            bundle = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(len(bundle["goals"]), 1)
            self.assertEqual(bundle["goals"][0]["kind"], "observation")
            self.assertAlmostEqual(bundle["goals"][0]["yaw"], 1.2)
            self.assertTrue(bundle["safety"]["start_pose_explicitly_approved"])
            self.assertFalse(bundle["safety"]["autonomous_semantic_execution"])


if __name__ == "__main__":
    unittest.main()
