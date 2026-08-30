from pathlib import Path
import unittest

from scripts.write_stage2_bag_instruction import build_manifest


class Stage2BagInstructionTest(unittest.TestCase):
    def test_manifest_keeps_instruction_and_compact_task_identity(self):
        graph = {
            "instruction": "检查桌上的杯子。",
            "tasks": [{
                "id": "check_cup", "action": "inspect",
                "verification_label": "cup",
                "target": {"label": "cup", "reference": "table"},
                "spatial_constraints": {"distance_m": None},
            }],
        }
        value = build_manifest(graph, Path("demo.bag"))
        self.assertEqual(value["bag_filename"], "demo.bag")
        self.assertEqual(value["instruction"], "检查桌上的杯子。")
        self.assertEqual(value["tasks"][0]["id"], "check_cup")
        self.assertNotIn("spatial_constraints", value["tasks"][0])


if __name__ == "__main__":
    unittest.main()
