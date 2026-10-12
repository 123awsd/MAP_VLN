from __future__ import annotations

import unittest

from stage2.vlm_mission_evaluator import normalize_mission_evaluation


class VlmMissionEvaluatorTest(unittest.TestCase):
    def test_normalizes_generic_multitask_result(self):
        result = normalize_mission_evaluation({
            "instruction_summary": "检查两个位置",
            "tasks": [
                {
                    "task_index": 1,
                    "request": "检查桌上的杯子",
                    "image_indices": [1],
                    "completion_status": "completed",
                    "observation_result": "present",
                    "confidence": 0.9,
                    "evidence": "桌面可见杯子",
                },
                {
                    "task_index": 2,
                    "request": "未触发的备用搜索",
                    "image_indices": [],
                    "completion_status": "skipped",
                    "observation_result": "not_triggered",
                    "confidence": 0.95,
                    "evidence": "主目标已找到，因此条件分支未触发",
                },
            ],
            "overall_status": "completed",
            "overall_reply": "杯子还在，备用搜索没有触发。",
            "limitations": [],
        }, 1)
        self.assertEqual(result["tasks"][1]["completion_status"], "skipped")
        self.assertEqual(result["overall_status"], "completed")

    def test_rejects_missing_image_reference(self):
        with self.assertRaises(ValueError):
            normalize_mission_evaluation({
                "tasks": [{
                    "request": "检查目标",
                    "image_indices": [2],
                    "completion_status": "completed",
                    "observation_result": "present",
                    "confidence": 0.9,
                    "evidence": "可见",
                }],
                "overall_status": "completed",
                "overall_reply": "完成",
            }, 1)


if __name__ == "__main__":
    unittest.main()
