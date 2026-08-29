from __future__ import annotations

import unittest

from stage2.task_graph_scene_validation import TaskSceneValidationError, validate_task_graph_against_scene


class TaskGraphSceneValidationTest(unittest.TestCase):
    def setUp(self):
        self.scene = {"rooms": [{"id": "L2_R1", "floor_id": 2, "objects": [
            {"id": "tv", "label": "television"}, {"id": "fire", "label": "fireplace"},
        ]}]}

    def test_accepts_scoped_target_and_reference(self):
        graph = {"tasks": [{"id": "t", "target": {
            "label": "television", "floor_id": 2, "references": ["fireplace"],
        }}]}
        result = validate_task_graph_against_scene(graph, self.scene)
        self.assertEqual(result["tasks"]["t"]["reference_object_ids"], ["fire"])

    def test_rejects_reference_on_wrong_floor(self):
        graph = {"tasks": [{"id": "t", "target": {
            "label": "television", "floor_id": 2, "references": ["bed"],
        }}]}
        with self.assertRaises(TaskSceneValidationError):
            validate_task_graph_against_scene(graph, self.scene)


if __name__ == "__main__":
    unittest.main()
