from __future__ import annotations

import unittest

from stage2.task_graph import TaskGraphError, normalize_and_validate_task_graph


class TaskGraphTest(unittest.TestCase):
    def base_graph(self):
        return {
            "tasks": [
                {
                    "id": "inspect_kitchen_cup",
                    "action": "inspect",
                    "target": {"label": "cup", "room": "kitchen"},
                    "prerequisites": ["deliver_folder"],
                },
                {
                    "id": "deliver_folder",
                    "action": "deliver",
                    "target": {"label": "desk", "room": "office", "reference": "folder"},
                },
                {
                    "id": "inspect_living_cup",
                    "action": "inspect",
                    "target": {"label": "cup", "room": "living_room"},
                },
            ],
            "conditional_rules": [
                {
                    "source_task_id": "inspect_kitchen_cup",
                    "if_outcome": "not_found",
                    "activate_task_ids": ["inspect_living_cup"],
                }
            ],
        }

    def test_normalizes_conditional_task(self):
        graph = normalize_and_validate_task_graph(self.base_graph(), "test")
        self.assertEqual(graph["summary"]["task_count"], 3)
        tasks = {task["id"]: task for task in graph["tasks"]}
        self.assertFalse(tasks["inspect_living_cup"]["active_initially"])
        self.assertTrue(tasks["inspect_kitchen_cup"]["active_initially"])
        self.assertEqual(tasks["deliver_folder"]["spatial_constraints"]["distance_m"], [1.0, 2.2])

    def test_rejects_cycle(self):
        value = self.base_graph()
        value["tasks"][1]["prerequisites"] = ["inspect_kitchen_cup"]
        with self.assertRaisesRegex(TaskGraphError, "cycle"):
            normalize_and_validate_task_graph(value)

    def test_rejects_unknown_conditional_target(self):
        value = self.base_graph()
        value["conditional_rules"][0]["activate_task_ids"] = ["missing"]
        with self.assertRaisesRegex(TaskGraphError, "unknown task"):
            normalize_and_validate_task_graph(value)

    def test_normalizes_string_none(self):
        value = self.base_graph()
        value["tasks"][0]["target"]["room"] = "none"
        value["tasks"][0]["target"]["reference"] = "null"
        graph = normalize_and_validate_task_graph(value)
        self.assertIsNone(graph["tasks"][0]["target"]["room"])
        self.assertIsNone(graph["tasks"][0]["target"]["reference"])

    def test_preserves_multifloor_target(self):
        value = self.base_graph()
        value["tasks"][0]["target"]["floor_id"] = 3
        graph = normalize_and_validate_task_graph(value)
        self.assertEqual(graph["tasks"][0]["target"]["floor_id"], 3)

    def test_rejects_invalid_multifloor_target(self):
        value = self.base_graph()
        value["tasks"][0]["target"]["floor_id"] = 0
        with self.assertRaisesRegex(TaskGraphError, "positive integer"):
            normalize_and_validate_task_graph(value)

    def test_accepts_on_surface_relation(self):
        value = self.base_graph()
        value["tasks"][0]["spatial_constraints"] = {"relation": "on"}
        graph = normalize_and_validate_task_graph(value)
        self.assertEqual(graph["tasks"][0]["spatial_constraints"]["relation"], "on")
        self.assertEqual(graph["tasks"][0]["spatial_constraints"]["region_type"], "auto")
        self.assertEqual(graph["tasks"][0]["spatial_constraints"]["observation_detail"], "normal")

    def test_normalizes_between_references_and_height_constraints(self):
        value = self.base_graph()
        value["tasks"][0]["target"]["references"] = ["table", "sofa"]
        value["tasks"][0]["spatial_constraints"] = {
            "relation": "between",
            "height_range_m": [0.7, 1.8],
            "region_type": "between_region",
            "observation_detail": "fine",
        }
        graph = normalize_and_validate_task_graph(value)
        task = graph["tasks"][0]
        self.assertEqual(task["target"]["references"], ["table", "sofa"])
        self.assertEqual(task["target"]["reference"], "table")
        self.assertEqual(task["target"]["reference_secondary"], "sofa")
        self.assertEqual(task["spatial_constraints"]["height_range_m"], [0.7, 1.8])
        self.assertEqual(task["spatial_constraints"]["region_type"], "between_region")

    def test_rejects_between_without_two_references(self):
        value = self.base_graph()
        value["tasks"][0]["spatial_constraints"] = {"relation": "between"}
        with self.assertRaisesRegex(TaskGraphError, "requires two reference"):
            normalize_and_validate_task_graph(value)

    def test_preserves_explicit_verification_label(self):
        value = self.base_graph()
        value["tasks"][0]["target"]["label"] = "counter"
        value["tasks"][0]["verification_label"] = "cup"
        graph = normalize_and_validate_task_graph(value)
        self.assertEqual(graph["tasks"][0]["verification_label"], "cup")

    def test_normalizes_semantic_recovery_policy(self):
        value = self.base_graph()
        value["tasks"][0]["search_policy"] = {
            "mode": "semantic_recovery", "maximum_location_hypotheses": 9,
        }
        graph = normalize_and_validate_task_graph(value)
        policy = graph["tasks"][0]["search_policy"]
        self.assertEqual(policy["completion_policy"], "first_success")
        self.assertEqual(policy["maximum_location_hypotheses"], 5)


if __name__ == "__main__":
    unittest.main()
