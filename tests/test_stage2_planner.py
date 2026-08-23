from __future__ import annotations

import unittest

import numpy as np

from stage2.grid_map import OccupancyGrid
from stage2.joint_planner import plan_joint_mission


class PlannerTest(unittest.TestCase):
    def test_astar_routes_around_wall(self):
        data = np.zeros((20, 20), dtype=np.int8)
        data[:, 10] = 100
        data[10, 10] = 0
        grid = OccupancyGrid(data, [0, 0], 1.0, inflation_m=0.0)
        path = grid.astar([2.5, 2.5], [17.5, 2.5])
        self.assertIsNotNone(path)
        self.assertGreater(path.length_m, 15.0)

    def test_subset_dp_reorders_independent_tasks(self):
        grid = OccupancyGrid(np.zeros((10, 30), dtype=np.int8), [0, 0], 1.0, inflation_m=0.0)
        graph = {
            "tasks": [
                {"id": "far", "active_initially": True, "prerequisites": []},
                {"id": "near", "active_initially": True, "prerequisites": []},
            ]
        }
        def candidate(task, x):
            return {"id": task + "_pose", "task_id": task, "object_id": task, "pose": {"x": x, "y": 2.5, "z": 1, "yaw": 0}, "terminal_cost": 0}
        result = plan_joint_mission(grid, graph, {"far": [candidate("far", 20.5)], "near": [candidate("near", 5.5)]}, [1.5, 2.5, 1, 0])
        self.assertEqual([visit["task_id"] for visit in result["visits"]], ["near", "far"])

    def test_subset_dp_honors_precedence(self):
        grid = OccupancyGrid(np.zeros((10, 30), dtype=np.int8), [0, 0], 1.0, inflation_m=0.0)
        graph = {"tasks": [
            {"id": "far", "active_initially": True, "prerequisites": []},
            {"id": "near", "active_initially": True, "prerequisites": ["far"]},
        ]}
        candidates = {
            "far": [{"id": "f", "object_id": "f", "pose": {"x": 20.5, "y": 2.5, "z": 1, "yaw": 0}, "terminal_cost": 0}],
            "near": [{"id": "n", "object_id": "n", "pose": {"x": 5.5, "y": 2.5, "z": 1, "yaw": 0}, "terminal_cost": 0}],
        }
        result = plan_joint_mission(grid, graph, candidates, [1.5, 2.5, 1, 0])
        self.assertEqual([visit["task_id"] for visit in result["visits"]], ["far", "near"])


if __name__ == "__main__":
    unittest.main()
