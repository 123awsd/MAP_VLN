from __future__ import annotations

import unittest
from collections import Counter

import numpy as np

from stage2.benchmark_dataset import build_scene_episodes, validate_distribution
from stage2.grid_map import OccupancyGrid


class BenchmarkDatasetTest(unittest.TestCase):
    @staticmethod
    def synthetic_scene():
        labels = [
            "chair", "chair", "table", "sofa", "bed", "cabinet", "desk",
            "painting", "bathtub", "mirror", "stool", "stool",
        ]
        rooms = []
        for index, label in enumerate(labels):
            x = 4.0 + (index % 4) * 8.0
            y = 4.0 + (index // 4) * 10.0
            rooms.append({
                "id": index + 1, "space_role": "room", "semantic_type": "unknown",
                "objects": [{
                    "id": f"obj_{index}", "label": label,
                    "center_xyz_m": [x, y, 0.8], "size_xyz_m": [0.5, 0.5, 1.0],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0], "probability": 0.8,
                }],
            })
        return {"rooms": rooms}

    def test_scene_episodes_are_executable_long_tasks(self):
        grid = OccupancyGrid(np.zeros((40, 40), dtype=np.int8), [0, 0], 1.0, inflation_m=0.0)
        episodes = build_scene_episodes("synthetic", self.synthetic_scene(), grid, 0)
        self.assertEqual(len(episodes), 10)
        for episode in episodes:
            graph = episode["task_graph"]
            expected_count = 5 if episode["category"] == "ordered_conditional" else 4
            self.assertEqual(graph["summary"]["task_count"], expected_count)
            self.assertGreaterEqual(graph["summary"]["precedence_edge_count"], 3)
            self.assertIn("先", graph["instruction"])
            self.assertIn("最后", graph["instruction"])
            self.assertEqual(len(set(episode["selected_objects"].values())), expected_count)
        for episode in episodes[7:]:
            self.assertIn("历史位置可能已经不准确", episode["task_graph"]["instruction"])
            self.assertTrue(episode["controls"]["stale_object_ids"])
            self.assertEqual(
                episode["controls"]["controlled_recovery_task_outcomes"],
                {"goal_2": episode["controls"]["expected_recovery"]},
            )

    def test_final_distribution(self):
        episodes = []
        for scene in range(10):
            for index in range(4):
                episodes.append({"category": "ordered_spatial", "controls": {
                    "expected_branch": None, "expected_recovery": None}})
            for index in range(3):
                episodes.append({"category": "ordered_conditional", "controls": {
                    "expected_branch": "taken" if (scene * 3 + index) % 2 == 0 else "skipped",
                    "expected_recovery": None}})
            for index in range(3):
                episodes.append({"category": "ordered_recovery", "controls": {
                    "expected_branch": None,
                    "expected_recovery": "rediscovered" if index < 2 else "exhausted"}})
        summary = validate_distribution(episodes, 10)
        self.assertEqual(summary["episode_count"], 100)
        self.assertEqual(Counter(summary["branches"]), Counter({"taken": 15, "skipped": 15}))
        self.assertEqual(summary["recoveries"], {"rediscovered": 20, "exhausted": 10})

    def test_seven_scene_pilot_distribution(self):
        episodes = []
        for scene in range(7):
            for _ in range(4):
                episodes.append({"category": "ordered_spatial", "controls": {
                    "expected_branch": None, "expected_recovery": None}})
            for index in range(3):
                episodes.append({"category": "ordered_conditional", "controls": {
                    "expected_branch": "taken" if (scene * 3 + index) % 2 == 0 else "skipped",
                    "expected_recovery": None}})
            for index in range(3):
                episodes.append({"category": "ordered_recovery", "controls": {
                    "expected_branch": None,
                    "expected_recovery": "rediscovered" if index < 2 else "exhausted"}})
        summary = validate_distribution(episodes, 7)
        self.assertEqual(summary["episode_count"], 70)
        self.assertEqual(summary["categories"], {
            "ordered_spatial": 28, "ordered_conditional": 21, "ordered_recovery": 21,
        })
        self.assertEqual(summary["branches"], {"taken": 11, "skipped": 10})
        self.assertEqual(summary["recoveries"], {"rediscovered": 14, "exhausted": 7})


if __name__ == "__main__":
    unittest.main()
