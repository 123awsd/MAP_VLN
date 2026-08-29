from __future__ import annotations

import unittest

import numpy as np

from stage2.grid_map import OccupancyGrid
from stage2.mission_executor import MissionState, simulate_dynamic_execution


def graph():
    return {
        "tasks": [
            {"id": "inspect_a", "active_initially": True, "prerequisites": [], "success_outcome": "found"},
            {"id": "fallback_b", "active_initially": False, "prerequisites": [], "success_outcome": "found"},
        ],
        "conditional_rules": [{"source_task_id": "inspect_a", "if_outcome": "not_found", "activate_task_ids": ["fallback_b"], "skip_task_ids": []}],
    }


def candidates():
    def value(task, x):
        return {"id": task + "_pose", "object_id": task, "pose": {"x": x, "y": 2.5, "z": 1.0, "yaw": 0.0}, "terminal_cost": 0.0}
    return {"inspect_a": [value("inspect_a", 4.5)], "fallback_b": [value("fallback_b", 8.5)]}


class ExecutorTest(unittest.TestCase):
    def test_matching_condition_activates_fallback(self):
        state = MissionState(graph())
        state.finish_task("inspect_a", "not_found")
        self.assertEqual(state.status["fallback_b"], "pending")

    def test_nonmatching_condition_skips_fallback(self):
        state = MissionState(graph())
        state.finish_task("inspect_a", "found")
        self.assertTrue(state.is_finished())
        self.assertEqual(state.status["fallback_b"], "skipped")

    def test_negative_branch_is_only_complete_after_fallback_success(self):
        state = MissionState(graph())
        state.finish_task("inspect_a", "not_found")
        self.assertEqual(state.result_status(), "incomplete")
        state.finish_task("fallback_b", "found")
        self.assertEqual(state.result_status(), "complete")
        self.assertEqual(state.unresolved_task_ids(), [])

    def test_unresolved_find_is_not_reported_complete(self):
        value = graph()
        value["conditional_rules"] = []
        value["tasks"] = [value["tasks"][0]]
        state = MissionState(value)
        state.finish_task("inspect_a", "not_found")
        self.assertEqual(state.result_status(), "completed_with_unresolved_tasks")
        self.assertEqual(state.unresolved_task_ids(), ["inspect_a"])

    def test_execution_replans_for_new_branch(self):
        grid = OccupancyGrid(np.zeros((10, 15), dtype=np.int8), [0, 0], 1.0, inflation_m=0)
        trace = simulate_dynamic_execution(grid, graph(), candidates(), [1.5, 2.5, 1, 0], {"inspect_a": "not_found"})
        self.assertEqual(trace["status"], "complete")
        self.assertEqual([visit["task_id"] for visit in trace["executed_visits"]], ["inspect_a", "fallback_b"])
        self.assertEqual(len(trace["replans"]), 2)


if __name__ == "__main__":
    unittest.main()
