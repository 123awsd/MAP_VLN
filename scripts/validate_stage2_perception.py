#!/usr/bin/env python3
"""Acceptance checks for asynchronous perception, fusion, recovery and pilot metrics."""

from __future__ import annotations

import argparse
import collections
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--online-run", type=Path, default=ROOT / "outputs/stage2/hm3d_perception_v2/habitat_demo")
    parser.add_argument("--recovery-run", type=Path, default=ROOT / "outputs/stage2/hm3d_recovery_test/habitat_demo")
    parser.add_argument("--multiscene-report", type=Path, default=ROOT / "outputs/stage2/open_vocab_multiscene/report.json")
    parser.add_argument("--bag-manifest", type=Path, default=ROOT / "outputs/bags/hm3d_stage2_perception_v2_complete.manifest.json")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/stage2/perception_v2_report.json")
    args = parser.parse_args()
    online = load_json(args.online_run / "habitat_execution.json")
    recovery = load_json(args.recovery_run / "habitat_execution.json")
    benchmark = load_json(args.multiscene_report)
    bag = load_json(args.bag_manifest)

    async_stats = online["open_vocab_detector"]["async"]
    fusion = online["semantic_fusion"]
    require(online["status"] == "completed", "online fused mission is incomplete")
    require(async_stats["submitted"] >= 8, "too few asynchronous detector submissions")
    require(len(online["open_vocab_observations"]) == async_stats["submitted"], "an asynchronous result was lost")
    require(len(fusion["confirmed_tracks"]) >= 8, "too few fused semantic tracks")
    require(all(item["support_count"] >= 2 for item in fusion["tracks"] if item["source"] == "online_novel" and item["confirmed"]), "a single-view novel object was accepted")
    require(bag["topic_counts"]["/stage2/open_vocab_boxes"] == len(online["open_vocab_observations"]), "fused box bag events are incomplete")

    outcomes = collections.Counter(item["outcome"] for item in recovery["observations"])
    attempts = collections.defaultdict(list)
    for item in recovery["observations"]:
        attempts[item["task_id"]].append(item["recovery"]["attempt_index"])
    require(recovery["status"] == "completed", "controlled recovery run is incomplete")
    require(outcomes["retry"] >= 4, "alternative viewpoints were not exercised")
    require(all(values == [1, 2] for values in attempts.values()), "recovery did not enforce exactly two controlled attempts")
    require(recovery["task_status"]["inspect_cabinet"] == "completed", "not-found branch did not complete cabinet recovery")

    protocol = benchmark["protocol"]
    micro = benchmark["image_level_semantic_gt"]["micro"]
    require(benchmark["status"] == "passed", "multi-scene perception benchmark failed")
    require(len(protocol["scenes"]) == 3 and protocol["views_per_scene"] == 48, "benchmark does not contain 144 real views")
    require(protocol["quantitative_gt_scenes"] == ["00861-GLAQ4DNUx5U"], "semantic-GT boundary changed")
    require(benchmark["latency_ms"]["p95"] < 60.0, "OWLv2 p95 latency exceeds real-time budget")
    require(micro["f1"] > 0.50, "single-frame pilot F1 is unexpectedly low")

    report = {
        "format": "pre_map_vln.perception_v2_validation.v1", "status": "passed",
        "online": {
            "task_count": len(online["observations"]), "frame_count": online["frame_count"],
            "path_length_m": online["path_length_m"], "async": async_stats,
            "fused_track_count": len(fusion["tracks"]),
            "confirmed_track_count": len(fusion["confirmed_tracks"]),
            "unconfirmed_novel_count": sum(item["source"] == "online_novel" and not item["confirmed"] for item in fusion["tracks"]),
            "added_novel_object_ids": fusion["added_novel_object_ids"],
        },
        "recovery": {
            "visit_count": len(recovery["observations"]), "retry_count": outcomes["retry"],
            "path_length_m": recovery["path_length_m"], "attempts_by_task": dict(attempts),
        },
        "perception_benchmark": {
            "view_count": len(benchmark["records"]), "latency_ms": benchmark["latency_ms"],
            "micro": micro,
            "recommended_global_threshold": benchmark["image_level_semantic_gt"]["recommended_global_threshold"],
            "semantic_gt_scenes": protocol["quantitative_gt_scenes"],
        },
        "bag": bag,
    }
    atomic_json(args.output, report)
    print(f"status=passed views={report['perception_benchmark']['view_count']} f1={micro['f1']:.3f} p95={benchmark['latency_ms']['p95']:.1f}ms retries={outcomes['retry']}")


if __name__ == "__main__":
    main()
