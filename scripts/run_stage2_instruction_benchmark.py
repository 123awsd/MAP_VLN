#!/usr/bin/env python3
"""Run natural-language Stage-2 cases sequentially and score real executions."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SUITE = ROOT / "config/stage2_benchmarks/00337_functional_v1.json"


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def normalized_label(value: Any) -> str:
    return " ".join(str(value or "").strip().lower().replace("_", " ").split())


def ensure_prepared_bundle(suite: dict[str, Any], prepared_name: str) -> Path:
    """Create a zero-copy prepared view when the suite declares existing sources."""
    prepared_dir = ROOT / "outputs/stage2_3d/prepared" / prepared_name
    manifest = prepared_dir / "manifest.json"
    if manifest.is_file():
        return prepared_dir
    if prepared_dir.exists():
        raise RuntimeError(f"incomplete prepared bundle already exists: {prepared_dir}")
    sources = suite.get("prepared_sources") or {}
    required = {
        "scene_graph": ROOT / str(sources.get("scene_graph", "")),
        "voxel_snapshot": ROOT / str(sources.get("voxel_snapshot", "")),
        "generated_stage1_config": ROOT / str(sources.get("generated_stage1_config", "")),
    }
    missing = []
    for key, path in required.items():
        if key == "voxel_snapshot":
            valid = (path / "metadata.json").is_file() and (path / "voxel_map.npz").is_file()
        else:
            valid = path.is_file()
        if not valid:
            missing.append(str(path))
    if missing:
        raise RuntimeError(
            "prepared bundle is missing and declared sources are unavailable: "
            + ", ".join(missing)
        )
    prepared_dir.mkdir(parents=True)
    for name, source in (
        ("scene_graph.json", required["scene_graph"]),
        ("voxel_snapshot", required["voxel_snapshot"]),
        ("generated_stage1_config.json", required["generated_stage1_config"]),
    ):
        (prepared_dir / name).symlink_to(os.path.relpath(source, prepared_dir))
    write_json(manifest, {
        "format": "pre_map_vln.prepared_multifloor_stage2.v1",
        "name": prepared_name,
        "scene_id": suite["scene_id"],
        "scene_graph": "scene_graph.json",
        "voxel_snapshot": "voxel_snapshot",
        "generated_stage1_config": "generated_stage1_config.json",
        "storage": "zero-copy links materialized by the instruction benchmark runner",
    })
    return prepared_dir


def ordered_subsequence(required: list[str], actual: list[str]) -> bool:
    cursor = 0
    for label in actual:
        if cursor < len(required) and label == required[cursor]:
            cursor += 1
    return cursor == len(required)


def evaluate_case(
    case: dict[str, Any], task_graph_path: Path, execution_path: Path,
    process_returncode: int, log_path: Path | None = None,
) -> dict[str, Any]:
    expected = case.get("expected", {})
    checks: dict[str, bool] = {"process_returncode_zero": process_returncode == 0}
    if not task_graph_path.is_file() or not execution_path.is_file():
        log_text = (
            log_path.read_text(encoding="utf-8", errors="replace")
            if log_path is not None and log_path.is_file() else ""
        )
        infrastructure_error = any(pattern in log_text for pattern in (
            "missing HM3D input:", "missing prepared input:",
            "prepared bundle is missing", "missing input:",
        ))
        checks["required_outputs_exist"] = False
        return {
            "id": case["id"], "category": case.get("category"), "passed": False,
            "evaluable": not infrastructure_error,
            "failure_kind": "infrastructure_error" if infrastructure_error else "execution_failure",
            "checks": checks, "required_target_count": len(expected.get("required_found_labels", [])),
            "found_required_target_count": 0,
            "error": "task_graph.json or habitat_execution.json is missing",
        }

    task_graph = load_json(task_graph_path)
    execution = load_json(execution_path)
    tasks = {task["id"]: task for task in task_graph.get("tasks", [])}
    event_rows = [event for event in execution.get("events", []) if event.get("event") == "task_finished"]
    executed = [
        normalized_label(tasks.get(event.get("task_id"), {}).get("verification_label"))
        for event in event_rows
    ]
    found = [
        normalized_label(tasks.get(event.get("task_id"), {}).get("verification_label"))
        for event in event_rows if event.get("outcome") == "found"
    ]
    required = [normalized_label(label) for label in expected.get("required_found_labels", [])]
    forbidden = [normalized_label(label) for label in expected.get("forbidden_executed_labels", [])]
    required_order = [normalized_label(label) for label in expected.get("required_found_order", [])]
    relations = {
        normalized_label(task.get("spatial_constraints", {}).get("relation"))
        for task in tasks.values()
    }
    floors = {
        int(task.get("target", {}).get("floor_id"))
        for task in tasks.values() if task.get("target", {}).get("floor_id") is not None
    }
    recovery = execution.get("semantic_recovery", {}) or {}
    recovery_events = recovery.get("events", []) or []
    clearance = execution.get("clearance_audit", {}) or {}
    hard_distance = float(clearance.get("hard_distance_m", 0.0) or 0.0)
    minimum_esdf = float(clearance.get("minimum_esdf_m", 0.0) or 0.0)

    checks.update({
        "required_outputs_exist": True,
        "execution_complete": execution.get("status") == "complete",
        "no_unresolved_tasks": not execution.get("unresolved_task_ids", []),
        "planning_is_3d": execution.get("planning_dimension") == "3d",
        "all_required_targets_found": all(label in found for label in required),
        "forbidden_targets_not_executed": all(label not in executed for label in forbidden),
        "collision_free": (
            int(clearance.get("occupied_or_unknown_pose_count", -1)) == 0
            and minimum_esdf + 1e-6 >= hard_distance
        ),
    })
    if "task_count" in expected:
        checks["task_count"] = len(tasks) == int(expected["task_count"])
    if expected.get("required_relations"):
        checks["required_relations"] = all(
            normalized_label(item) in relations for item in expected["required_relations"]
        )
    if required_order:
        checks["required_found_order"] = ordered_subsequence(required_order, found)
    if expected.get("required_floors"):
        checks["required_floors"] = set(map(int, expected["required_floors"])) <= floors
    if "minimum_precedence_edges" in expected:
        edge_count = sum(len(task.get("prerequisites", [])) for task in tasks.values())
        checks["minimum_precedence_edges"] = edge_count >= int(expected["minimum_precedence_edges"])
    if "minimum_conditional_rules" in expected:
        checks["minimum_conditional_rules"] = len(task_graph.get("conditional_rules", [])) >= int(
            expected["minimum_conditional_rules"]
        )
    if expected.get("semantic_recovery"):
        checks["semantic_recovery_enabled"] = bool(recovery.get("enabled"))
        checks["minimum_recovery_events"] = len(recovery_events) >= int(
            expected.get("minimum_recovery_events", 1)
        )

    found_required = sum(label in found for label in required)
    return {
        "id": case["id"],
        "category": case.get("category"),
        "passed": all(checks.values()),
        "evaluable": True,
        "failure_kind": None if all(checks.values()) else "benchmark_failure",
        "checks": checks,
        "execution_status": execution.get("status"),
        "task_count": len(tasks),
        "executed_labels": executed,
        "found_labels": found,
        "required_labels": required,
        "required_target_count": len(required),
        "found_required_target_count": found_required,
        "unresolved_task_ids": execution.get("unresolved_task_ids", []),
        "path_length_m": execution.get("path_length_m"),
        "elapsed_wall_s": execution.get("elapsed_wall_s"),
        "frame_count": execution.get("frame_count"),
        "minimum_esdf_m": clearance.get("minimum_esdf_m"),
        "hard_distance_m": clearance.get("hard_distance_m"),
        "recovery_event_count": len(recovery_events),
    }


def aggregate(suite: dict[str, Any], session: str, results: list[dict[str, Any]]) -> dict[str, Any]:
    attempted = len(results)
    evaluable_results = [item for item in results if item.get("evaluable", True)]
    total = len(evaluable_results)
    passed = sum(bool(item.get("passed")) for item in evaluable_results)
    targets = sum(int(item.get("required_target_count", 0)) for item in evaluable_results)
    found_targets = sum(int(item.get("found_required_target_count", 0)) for item in evaluable_results)
    collision_audited_results = [
        item for item in evaluable_results if "collision_free" in item.get("checks", {})
    ]
    collision_free = sum(
        bool(item["checks"]["collision_free"]) for item in collision_audited_results
    )
    return {
        "format": "pre_map_vln.stage2_instruction_benchmark_report.v1",
        "suite": suite["name"],
        "scene_id": suite["scene_id"],
        "session": session,
        "attempted_case_count": attempted,
        "infrastructure_error_count": attempted - total,
        "evaluable_case_count": total,
        "completed_case_count": total,
        "passed_case_count": passed,
        "instruction_success_rate": passed / total if total else None,
        "required_target_count": targets,
        "found_required_target_count": found_targets,
        "target_success_rate": found_targets / targets if targets else None,
        "collision_audited_case_count": len(collision_audited_results),
        "collision_free_case_count": collision_free,
        "collision_free_case_rate": (
            collision_free / len(collision_audited_results)
            if collision_audited_results else None
        ),
        "total_path_length_m": sum(float(item.get("path_length_m") or 0.0) for item in results),
        "total_elapsed_wall_s": sum(float(item.get("elapsed_wall_s") or 0.0) for item in results),
        "results": results,
    }


def write_csv(path: Path, results: list[dict[str, Any]]) -> None:
    fields = [
        "id", "category", "passed", "evaluable", "failure_kind",
        "execution_status", "task_count",
        "found_required_target_count", "required_target_count", "path_length_m",
        "elapsed_wall_s", "frame_count", "minimum_esdf_m", "recovery_event_count",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({field: result.get(field) for field in fields})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", type=Path, default=DEFAULT_SUITE)
    parser.add_argument("--prepared", default=None, help="override suite prepared_name")
    parser.add_argument("--session", default=None)
    parser.add_argument("--case", action="append", dest="case_ids", help="run only this case; repeatable")
    parser.add_argument("--list", action="store_true", help="list cases without executing")
    args = parser.parse_args()

    suite_path = args.suite.resolve()
    suite = load_json(suite_path)
    cases = suite.get("cases", [])
    if args.case_ids:
        requested = set(args.case_ids)
        cases = [case for case in cases if case["id"] in requested]
        missing = requested - {case["id"] for case in cases}
        if missing:
            parser.error(f"unknown case IDs: {', '.join(sorted(missing))}")
    if args.list:
        for case in cases:
            print(f"{case['id']}: {case.get('category', '-')}\n  {case['instruction']}")
        return 0
    if not cases:
        parser.error("suite contains no selected cases")

    prepared_name = args.prepared or suite.get("prepared_name")
    try:
        prepared_dir = ensure_prepared_bundle(suite, str(prepared_name))
    except RuntimeError as error:
        parser.error(str(error))
    print(f"prepared={prepared_dir}")
    prepared_config = load_json(prepared_dir / "generated_stage1_config.json")
    scene_path = Path(str(prepared_config.get("scene", ""))).resolve()
    scene_config_path = Path(str(prepared_config.get("scene_config", ""))).resolve()
    expected_scene_id = str(suite["scene_id"])
    if not scene_path.is_file() or scene_path.parent.name != expected_scene_id:
        parser.error(f"prepared scene is missing or has the wrong scene ID: {scene_path}")
    if not scene_config_path.is_file():
        parser.error(f"prepared scene config is missing: {scene_config_path}")
    scene_root = scene_path.parent.parent
    child_environment = os.environ.copy()
    child_environment["PRE_MAP_VLN_HM3D_TRAIN_ROOT"] = str(scene_root)
    child_environment["PRE_MAP_VLN_HM3D_SCENE_CONFIG"] = str(scene_config_path)
    print(f"hm3d_scene={scene_path}")

    session = args.session or datetime.now().strftime("%Y%m%d_%H%M%S")
    if not session.replace("_", "").replace("-", "").isalnum():
        parser.error("session may contain only letters, numbers, '-' and '_'")
    session_dir = ROOT / "outputs/stage2_3d/benchmarks" / suite["name"] / session
    if session_dir.exists():
        parser.error(f"session already exists: {session_dir}")
    runs_dir = session_dir / "runs"
    runs_dir.mkdir(parents=True)
    shutil.copy2(suite_path, session_dir / "suite.json")

    results: list[dict[str, Any]] = []
    for index, case in enumerate(cases, 1):
        case_id = case["id"]
        temporary_name = f"bench_{suite['name']}_{session}_{case_id}"
        temporary_run = ROOT / "outputs/stage2_3d/tasks" / temporary_name
        final_run = runs_dir / case_id
        print(f"[{index}/{len(cases)}] {case_id}: {case['instruction']}", flush=True)
        command = [
            str(ROOT / "scripts/run_prepared_stage2.sh"), str(prepared_name),
            temporary_name, case["instruction"], suite.get("verification_mode", "owlv2"),
        ]
        log_path = session_dir / f"{case_id}.log"
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=child_environment,
            )
            assert process.stdout is not None
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(f"  | {line}", end="", flush=True)
            returncode = process.wait()
        if temporary_run.exists():
            shutil.move(str(temporary_run), str(final_run))
        result = evaluate_case(
            case, final_run / "task_graph.json",
            final_run / "execution/habitat_execution.json", returncode, log_path,
        )
        result["run_directory"] = str(final_run.relative_to(ROOT))
        result["log"] = str(log_path.relative_to(ROOT))
        results.append(result)
        report = aggregate(suite, session, results)
        write_json(session_dir / "report.json", report)
        write_csv(session_dir / "summary.csv", results)
        print(
            f"  {'PASS' if result['passed'] else 'FAIL'} "
            f"targets={result.get('found_required_target_count', 0)}/"
            f"{result.get('required_target_count', 0)}",
            flush=True,
        )

    report = aggregate(suite, session, results)
    if report["evaluable_case_count"]:
        print(
            f"instruction_success={report['passed_case_count']}/{report['evaluable_case_count']} "
            f"({100.0 * report['instruction_success_rate']:.1f}%) "
            f"target_success={report['found_required_target_count']}/{report['required_target_count']} "
            f"({100.0 * report['target_success_rate']:.1f}%)"
        )
    else:
        print(
            f"instruction_success=not_evaluable "
            f"infrastructure_errors={report['infrastructure_error_count']}"
        )
    print(f"report={session_dir / 'report.json'}")
    all_passed = (
        report["infrastructure_error_count"] == 0
        and report["evaluable_case_count"] > 0
        and report["passed_case_count"] == report["evaluable_case_count"]
    )
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
