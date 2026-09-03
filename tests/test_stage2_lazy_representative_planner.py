from __future__ import annotations

import unittest

import numpy as np

from stage2.grid_map import OccupancyGrid
from stage2.lazy_representative_planner import LazyRepresentativePlanner
from stage2.viewpoint_recovery import ViewpointRecovery


def candidate(identifier: str, x: float, terminal: float = 0.0) -> dict:
    return {
        "id": identifier,
        "object_id": identifier,
        "location_hypothesis_id": "test_location",
        "pose": {"x": x, "y": 2.5, "z": 1.0, "yaw": 0.0},
        "terminal_cost": terminal,
    }


class LazyRepresentativePlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.grid = OccupancyGrid(
            np.zeros((10, 30), dtype=np.int8), [0, 0], 1.0, inflation_m=0.0
        )

    def test_global_plan_evaluates_only_route_edges(self):
        graph = {
            "tasks": [
                {"id": "a", "active_initially": True, "prerequisites": []},
                {"id": "b", "active_initially": True, "prerequisites": []},
                {"id": "c", "active_initially": True, "prerequisites": []},
                {"id": "d", "active_initially": True, "prerequisites": []},
            ]
        }
        candidates = {
            "a": [candidate("a_primary", 4.5), candidate("a_backup", 4.5, 1.0)],
            "b": [candidate("b_primary", 8.5), candidate("b_backup", 8.5, 1.0)],
            "c": [candidate("c_primary", 14.5), candidate("c_backup", 14.5, 1.0)],
            "d": [candidate("d_primary", 20.5), candidate("d_backup", 20.5, 1.0)],
        }
        planner = LazyRepresentativePlanner(self.grid)
        result = planner.plan_global(graph, candidates, [1.5, 2.5, 1.0, 0.0])

        self.assertEqual(len(result["visits"]), 4)
        self.assertEqual(
            [item["candidate_id"] for item in result["visits"]],
            ["a_primary", "b_primary", "c_primary", "d_primary"],
        )
        self.assertTrue(result["lazy"]["certified"])
        self.assertLessEqual(result["lazy"]["edge_evaluations"], 10)
        self.assertEqual(result["lazy"]["edge_evaluations"], 4)

    def test_local_recovery_uses_next_candidate_and_keeps_one_visit(self):
        planner = LazyRepresentativePlanner(self.grid)
        result = planner.plan_local_recovery(
            "inspect_a",
            [candidate("a_backup", 5.5), candidate("a_backup_2", 7.5)],
            [1.5, 2.5, 1.0, 0.0],
        )
        self.assertEqual(len(result["visits"]), 1)
        self.assertEqual(result["visits"][0]["task_id"], "inspect_a")
        self.assertEqual(result["visits"][0]["candidate_id"], "a_backup")
        self.assertEqual(result["lazy"]["mode"], "local_viewpoint_recovery")

    def test_completed_prerequisite_is_allowed(self):
        graph = {
            "tasks": [
                {"id": "a", "active_initially": True, "prerequisites": []},
                {"id": "b", "active_initially": True, "prerequisites": ["a"]},
            ]
        }
        planner = LazyRepresentativePlanner(self.grid)
        result = planner.plan_global(
            graph,
            {"b": [candidate("b_primary", 5.5)]},
            [1.5, 2.5, 1.0, 0.0],
            active_task_ids={"b"},
            completed_task_ids={"a"},
        )
        self.assertEqual([item["task_id"] for item in result["visits"]], ["b"])

    def test_viewpoint_recovery_focuses_the_failed_task(self):
        values = [
            candidate("a_1", 4.5), candidate("a_2", 5.5), candidate("a_3", 6.5),
        ]
        recovery = ViewpointRecovery()
        decision = recovery.decide("a", "a_1", False, values)
        self.assertTrue(decision["retry"])
        filtered = recovery.filtered_candidates({"a": values})
        self.assertEqual(recovery.focused_task_ids({"a"}, filtered), {"a"})
        planner = LazyRepresentativePlanner(self.grid)
        result = planner.plan_local_recovery("a", filtered["a"], [4.5, 2.5, 1.0, 0.0])
        self.assertEqual(result["visits"][0]["candidate_id"], "a_2")


if __name__ == "__main__":
    unittest.main()
