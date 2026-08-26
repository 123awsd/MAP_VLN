#!/usr/bin/env python3
"""Aggregate auditable episode- and category-level benchmark metrics."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402


def mean(values):
    return statistics.fmean(values) if values else None


def episode_sha256(episode):
    encoded = json.dumps(episode, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def evaluate_one(root: Path, manifest_item: dict) -> dict:
    episode_id = manifest_item["episode_id"]
    prepared = root / "runs/prepared" / episode_id
    execution_path = root / "runs/executions" / episode_id / "habitat_execution.json"
    status_path = root / "runs/executions" / episode_id / "benchmark_status.json"
    episode = load_json(prepared / "episode.json")
    plan = load_json(prepared / "mission_plan.json")
    status_record = load_json(status_path) if status_path.exists() else {}
    if not execution_path.exists() or status_record.get("episode_sha256") != episode_sha256(episode):
        return {"episode_id": episode_id, "scene_id": episode["scene_id"],
                "category": episode["category"], "status": "missing_or_stale"}
    trace = load_json(execution_path)
    finished = [event for event in trace.get("events", []) if event.get("event") == "task_finished"]
    outcomes = {event["task_id"]: event["outcome"] for event in finished}
    sequence = [event["task_id"] for event in finished]
    task_by_id = {task["id"]: task for task in episode["task_graph"]["tasks"]}
    position = {task_id: index for index, task_id in enumerate(sequence)}
    order_ok = all(
        prerequisite in position and position[prerequisite] < position.get(task_id, 10**9)
        for task_id, task in task_by_id.items() if task_id in position
        for prerequisite in task["prerequisites"]
    )
    controls = episode["controls"]
    fallback_executed = "fallback_1" in outcomes
    expected_branch = controls.get("expected_branch")
    branch_ok = (
        None if expected_branch is None else
        fallback_executed == (expected_branch == "taken")
    )
    expected_recovery = controls.get("expected_recovery")
    recovery_events = trace.get("semantic_recovery", {}).get("events", [])
    recovery_task_outcome = outcomes.get("goal_2")
    recovery_ok = None
    if expected_recovery == "rediscovered":
        recovery_ok = bool(recovery_events) and recovery_task_outcome == "found"
    elif expected_recovery == "exhausted":
        recovery_ok = bool(recovery_events) and recovery_task_outcome == "not_found"
    required = [task["id"] for task in episode["task_graph"]["tasks"] if task["id"] != "fallback_1"]
    def expected_outcome(task_id):
        if expected_branch == "taken" and task_id == "goal_2":
            return "not_found"
        if expected_recovery == "exhausted" and task_id == "goal_2":
            return "not_found"
        return "found"
    expected_outcome_ok = all(
        outcomes.get(task_id) == expected_outcome(task_id)
        for task_id in required
    )
    if expected_branch == "taken":
        expected_outcome_ok = expected_outcome_ok and outcomes.get("fallback_1") == "found"
    detector_latencies = [
        float(item["latency_ms"]) for item in trace.get("open_vocab_observations", [])
        if item.get("latency_ms") is not None
    ]
    actual = float(trace.get("path_length_m", 0.0))
    optimal = float(plan.get("total_path_length_m", 0.0))
    path_efficiency = min(1.0, optimal / actual) if actual > 0 else 0.0
    constraints_ok = bool(
        order_ok and (branch_ok is not False) and (recovery_ok is not False)
    )
    mission_success = bool(expected_outcome_ok and constraints_ok)
    final_observation_by_task = {}
    for observation in trace.get("observations", []):
        if observation.get("outcome") in {"found", "not_found", "done", "skipped"}:
            final_observation_by_task[observation["task_id"]] = observation
    natural_terminal_results = []
    raw_open_vocab_results = []
    for observation in final_observation_by_task.values():
        verification = observation.get("verification", {})
        owl_result = verification.get("owlv2_found")
        if owl_result is not None:
            raw_open_vocab_results.append(bool(owl_result))
        controlled = bool(
            verification.get("controlled_stale_map")
            or verification.get("controlled_found")
            or verification.get("controlled_recovery_outcome")
        )
        if not controlled and owl_result is not None:
            natural_terminal_results.append(bool(owl_result))
    return {
        "episode_id": episode_id, "scene_id": episode["scene_id"],
        "category": episode["category"], "status": trace.get("status"),
        "expected_outcome_success": expected_outcome_ok,
        "mission_success": mission_success,
        "constraint_success": constraints_ok,
        "order_constraint_satisfied": order_ok,
        "branch_constraint_satisfied": branch_ok,
        "recovery_expectation_satisfied": recovery_ok,
        "finished_task_count": len(finished), "found_task_count": sum(v == "found" for v in outcomes.values()),
        "actual_path_length_m": actual, "initial_optimal_path_length_m": optimal,
        "path_efficiency": path_efficiency,
        "success_weighted_path_efficiency": path_efficiency if mission_success else 0.0,
        "elapsed_wall_s": float(trace.get("elapsed_wall_s", 0.0)),
        "frame_count": int(trace.get("frame_count", 0)),
        "detector_mean_latency_ms": mean(detector_latencies),
        "natural_terminal_detection_success_rate": mean(natural_terminal_results),
        "raw_open_vocab_terminal_success_rate": mean(raw_open_vocab_results),
        "semantic_recovery_event_count": len(recovery_events),
        "verification_mode": trace.get("verification_mode"),
    }


def summarize(rows):
    complete = [row for row in rows if row.get("status") == "completed"]
    def fraction(key, subset=complete):
        values = [bool(row[key]) for row in subset if row.get(key) is not None]
        return sum(values) / len(values) if values else None
    return {
        "episode_count": len(rows), "completed_count": len(complete),
        "expected_outcome_success_rate": fraction("expected_outcome_success"),
        "mission_success_rate": fraction("mission_success"),
        "constraint_success_rate": fraction("constraint_success"),
        "order_constraint_satisfaction_rate": fraction("order_constraint_satisfied"),
        "branch_constraint_satisfaction_rate": fraction("branch_constraint_satisfied"),
        "recovery_expectation_satisfaction_rate": fraction("recovery_expectation_satisfied"),
        "mean_path_length_m": mean([row["actual_path_length_m"] for row in complete]),
        "mean_path_efficiency": mean([row["path_efficiency"] for row in complete]),
        "mean_success_weighted_path_efficiency": mean([
            row["success_weighted_path_efficiency"] for row in complete
        ]),
        "mean_wall_time_s": mean([row["elapsed_wall_s"] for row in complete]),
        "mean_detector_latency_ms": mean([
            row["detector_mean_latency_ms"] for row in complete
            if row.get("detector_mean_latency_ms") is not None
        ]),
        "mean_natural_terminal_detection_success_rate": mean([
            row["natural_terminal_detection_success_rate"] for row in complete
            if row.get("natural_terminal_detection_success_rate") is not None
        ]),
        "mean_raw_open_vocab_terminal_success_rate": mean([
            row["raw_open_vocab_terminal_success_rate"] for row in complete
            if row.get("raw_open_vocab_terminal_success_rate") is not None
        ]),
    }


def write_markdown(path: Path, report: dict) -> None:
    metrics = [
        ("Episodes", "episode_count"), ("Completed", "completed_count"),
        ("Mission success", "mission_success_rate"),
        ("Constraint success", "constraint_success_rate"),
        ("Order satisfaction", "order_constraint_satisfaction_rate"),
        ("Branch satisfaction", "branch_constraint_satisfaction_rate"),
        ("Recovery satisfaction", "recovery_expectation_satisfaction_rate"),
        ("Path efficiency", "mean_path_efficiency"),
        ("Success-weighted efficiency", "mean_success_weighted_path_efficiency"),
        ("Natural detector success", "mean_natural_terminal_detection_success_rate"),
        ("Raw OWLv2 success", "mean_raw_open_vocab_terminal_success_rate"),
        ("Mean detector latency (ms)", "mean_detector_latency_ms"),
        ("Mean wall time (s)", "mean_wall_time_s"),
    ]

    def display(value):
        if value is None:
            return "—"
        if isinstance(value, float):
            return f"{value:.4f}"
        return str(value)

    lines = ["# PRE-MAP-VLN-Bench-v1 Results", "", "## Overall", "", "| Metric | Value |", "|---|---:|"]
    lines.extend(f"| {label} | {display(report['overall'].get(key))} |" for label, key in metrics)
    lines += ["", "## By category", "", "| Category | Episodes | Mission success | Constraint success | Path efficiency | Natural detector |", "|---|---:|---:|---:|---:|---:|"]
    for name, value in report["by_category"].items():
        lines.append(
            f"| {name} | {value['episode_count']} | {display(value['mission_success_rate'])} | "
            f"{display(value['constraint_success_rate'])} | {display(value['mean_path_efficiency'])} | "
            f"{display(value['mean_natural_terminal_detection_success_rate'])} |"
        )
    lines += ["", "## By scene", "", "| Scene | Completed | Mission success | Mean path (m) | Mean wall time (s) |", "|---|---:|---:|---:|---:|"]
    for name, value in report["by_scene"].items():
        lines.append(
            f"| {name} | {value['completed_count']}/{value['episode_count']} | "
            f"{display(value['mission_success_rate'])} | {display(value['mean_path_length_m'])} | "
            f"{display(value['mean_wall_time_s'])} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/shared/PRE_MAP_VLN_benchmark_v1"))
    parser.add_argument("--require-count", type=int, default=0)
    args = parser.parse_args()
    manifest = load_json(args.root / "tasks/manifest.json")
    rows = [evaluate_one(args.root, item) for item in manifest["episodes"]]
    categories = defaultdict(list)
    scenes = defaultdict(list)
    for row in rows:
        categories[row["category"]].append(row)
        scenes[row["scene_id"]].append(row)
    report = {
        "format": "pre_map_vln.benchmark_report.v1", "benchmark": manifest["name"],
        "overall": summarize(rows),
        "by_category": {key: summarize(value) for key, value in sorted(categories.items())},
        "by_scene": {key: summarize(value) for key, value in sorted(scenes.items())},
        "status_counts": dict(Counter(row["status"] for row in rows)),
        "episodes": rows,
    }
    report_dir = args.root / "reports"
    report_dir.mkdir(parents=True, exist_ok=True)
    atomic_json(report_dir / "benchmark_report.json", report)
    scalar_fields = sorted({
        key for row in rows for key, value in row.items()
        if isinstance(value, (str, int, float, bool)) or value is None
    })
    with (report_dir / "episodes.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=scalar_fields)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in scalar_fields} for row in rows)
    write_markdown(report_dir / "summary.md", report)
    print(json.dumps(report["overall"], ensure_ascii=False, indent=2))
    if args.require_count and report["overall"]["completed_count"] != args.require_count:
        raise SystemExit(f"expected {args.require_count} completed episodes")


if __name__ == "__main__":
    main()
