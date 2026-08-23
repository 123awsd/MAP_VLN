from __future__ import annotations

import unittest

import numpy as np

from stage2.benchmark_suite import build_task_suites, evaluate_method, planner_registry
from stage2.grid_map import OccupancyGrid


def candidate(task_id, index, x, terminal):
    return {
        "id": f"{task_id}_{index}", "task_id": task_id, "object_id": task_id,
        "pose": {"x": x, "y": 2.5, "z": 1.0, "yaw": 0.0},
        "terminal_cost": terminal, "line_of_sight": True,
    }


class BenchmarkSuiteTest(unittest.TestCase):
    def test_task_suites_cover_constraint_types(self):
        suites = build_task_suites()
        self.assertEqual(set(suites), {"independent", "precedence", "conditional", "spatial"})
        self.assertEqual(suites["conditional"]["summary"]["conditional_rule_count"], 1)
        self.assertGreater(suites["spatial"]["summary"]["task_count"], 3)

    def test_all_methods_share_metrics_and_finish(self):
        grid = OccupancyGrid(np.zeros((8, 35), dtype=np.int8), [0, 0], 1.0, inflation_m=0.0)
        graph = build_task_suites()["conditional"]
        positions = {"lamp": 5.5, "television": 20.5, "cabinet": 25.5, "door": 10.5, "chair": 15.5}
        candidates = {}
        for task in graph["tasks"]:
            x = positions[task["target"]["label"]]
            candidates[task["id"]] = [
                candidate(task["id"], 0, x, 0.0),
                candidate(task["id"], 1, x + 1.0, 0.2),
            ]
        for method, (group, planner) in planner_registry().items():
            result = evaluate_method(
                method, group, planner, grid, graph, candidates, [1.5, 2.5, 1.0, 0.0],
                outcomes={"inspect_tv": "not_found"},
            )
            self.assertTrue(result["mission_success"], method)
            self.assertTrue(result["condition_satisfaction"], method)
            self.assertEqual(result["constraint_satisfaction_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
