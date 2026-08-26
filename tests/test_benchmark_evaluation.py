from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts.evaluate_benchmark_v1 import episode_sha256, evaluate_one


def write(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def task(task_id, prerequisites=None):
    return {"id": task_id, "prerequisites": prerequisites or []}


class BenchmarkEvaluationTest(unittest.TestCase):
    def make_case(self, episode, events, observations, recovery_events=None):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        episode_id = episode["episode_id"]
        prepared = root / "runs/prepared" / episode_id
        execution = root / "runs/executions" / episode_id
        write(prepared / "episode.json", episode)
        write(prepared / "mission_plan.json", {"total_path_length_m": 8.0})
        write(execution / "benchmark_status.json", {
            "episode_sha256": episode_sha256(episode), "verification_mode": "owlv2",
        })
        write(execution / "habitat_execution.json", {
            "status": "completed", "events": events, "observations": observations,
            "semantic_recovery": {"events": recovery_events or []},
            "path_length_m": 10.0, "elapsed_wall_s": 3.0, "frame_count": 100,
            "open_vocab_observations": [{"latency_ms": 40.0}],
            "verification_mode": "owlv2",
        })
        return evaluate_one(root, {"episode_id": episode_id})

    def test_taken_condition_is_success_with_expected_negative_source(self):
        episode = {
            "episode_id": "scene_05", "scene_id": "scene", "category": "ordered_conditional",
            "task_graph": {"tasks": [
                task("goal_1"), task("goal_2", ["goal_1"]),
                task("goal_3", ["goal_2"]), task("fallback_1", ["goal_2"]),
            ]},
            "controls": {"expected_branch": "taken", "expected_recovery": None},
        }
        outcomes = [("goal_1", "found"), ("goal_2", "not_found"),
                    ("fallback_1", "found"), ("goal_3", "found")]
        events = [{"event": "task_finished", "task_id": key, "outcome": value}
                  for key, value in outcomes]
        observations = [{
            "task_id": key, "outcome": value,
            "verification": {
                "owlv2_found": value == "found", "controlled_stale_map": key == "goal_2",
                "controlled_found": key == "fallback_1", "controlled_recovery_outcome": None,
            },
        } for key, value in outcomes]
        row = self.make_case(episode, events, observations)
        self.assertTrue(row["mission_success"])
        self.assertTrue(row["branch_constraint_satisfied"])
        self.assertEqual(row["success_weighted_path_efficiency"], 0.8)

    def test_controlled_rediscovery_is_separate_from_raw_detector_metric(self):
        episode = {
            "episode_id": "scene_08", "scene_id": "scene", "category": "ordered_recovery",
            "task_graph": {"tasks": [
                task("goal_1"), task("goal_2", ["goal_1"]),
                task("goal_3", ["goal_2"]), task("goal_4", ["goal_3"]),
            ]},
            "controls": {"expected_branch": None, "expected_recovery": "rediscovered"},
        }
        outcomes = [(f"goal_{index}", "found") for index in range(1, 5)]
        events = [{"event": "task_finished", "task_id": key, "outcome": value}
                  for key, value in outcomes]
        observations = []
        for key, value in outcomes:
            observations.append({
                "task_id": key, "outcome": value,
                "verification": {
                    "owlv2_found": key != "goal_2", "controlled_stale_map": False,
                    "controlled_found": False,
                    "controlled_recovery_outcome": "rediscovered" if key == "goal_2" else None,
                },
            })
        row = self.make_case(episode, events, observations, recovery_events=[{"task_id": "goal_2"}])
        self.assertTrue(row["mission_success"])
        self.assertTrue(row["recovery_expectation_satisfied"])
        self.assertEqual(row["natural_terminal_detection_success_rate"], 1.0)
        self.assertEqual(row["raw_open_vocab_terminal_success_rate"], 0.75)


if __name__ == "__main__":
    unittest.main()
