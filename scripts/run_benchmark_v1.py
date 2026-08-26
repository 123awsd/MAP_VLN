#!/usr/bin/env python3
"""Resumable execution of prepared benchmark episodes in Habitat."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402


def episode_sha256(episode: dict) -> str:
    encoded = json.dumps(episode, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/shared/PRE_MAP_VLN_benchmark_v1"))
    parser.add_argument("--verification-mode", default="owlv2", choices=("semantic", "owlv2"))
    parser.add_argument("--only", nargs="*", default=[])
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--save-every", type=int, default=5)
    parser.add_argument("--owlv2-every", type=int, default=10)
    args = parser.parse_args()
    manifest = load_json(args.root / "tasks/manifest.json")
    wanted = set(args.only)
    items = [item for item in manifest["episodes"] if not wanted or item["episode_id"] in wanted]
    if args.limit:
        items = items[:args.limit]
    results = []
    started = time.monotonic()
    for index, item in enumerate(items, start=1):
        episode_id, scene_id = item["episode_id"], item["scene_id"]
        prepared = args.root / "runs/prepared" / episode_id
        output = args.root / "runs/executions" / episode_id
        result_path = output / "habitat_execution.json"
        episode = load_json(prepared / "episode.json")
        digest = episode_sha256(episode)
        status_path = output / "benchmark_status.json"
        status = load_json(status_path) if status_path.exists() else {}
        existing_result = load_json(result_path) if result_path.exists() else {}
        reusable = bool(
            result_path.exists()
            and status.get("episode_sha256") == digest
            and status.get("verification_mode") == args.verification_mode
            and status.get("return_code") == 0
            and existing_result.get("status") == "completed"
        )
        if reusable and not args.rerun:
            result = existing_result
            print(f"[{index}/{len(items)}] SKIP {episode_id} {result.get('status')}")
            results.append({"episode_id": episode_id, "status": result.get("status"), "skipped": True})
            continue
        if output.exists():
            snapshot_root = args.root / "runs/execution_snapshots" / episode_id
            snapshot_root.mkdir(parents=True, exist_ok=True)
            snapshot = snapshot_root / time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
            suffix = 1
            while snapshot.exists():
                snapshot = snapshot_root / f"{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{suffix}"
                suffix += 1
            shutil.move(str(output), str(snapshot))
            print(f"[{index}/{len(items)}] ARCHIVE {episode_id} -> {snapshot}")
        stage1_manifest = args.root / "episodes" / scene_id / "manifest.json"
        stage1 = load_json(stage1_manifest)
        scene = Path(stage1["scene_path"])
        scene_config = stage1.get("scene_dataset_config")
        output.mkdir(parents=True, exist_ok=True)
        controls = episode["controls"]
        command = [
            sys.executable, str(ROOT / "scripts/run_stage2_habitat.py"),
            "--task-graph", str(prepared / "task_graph.json"),
            "--candidates", str(prepared / "candidates.json"),
            "--scene-graph", str(args.root / "maps/scene_graph" / f"{scene_id}.json"),
            "--grid-prefix", str(args.root / "maps/grids" / f"{scene_id}_navigation"),
            "--scene", str(scene),
            "--episode-manifest", str(stage1_manifest),
            "--output-dir", str(output), "--verification-mode", args.verification_mode,
            "--save-every", str(args.save_every), "--owlv2-every", str(args.owlv2_every),
            "--max-viewpoint-attempts", "2",
            "--controlled-stale-object-ids", json.dumps(controls["stale_object_ids"]),
            "--controlled-found-object-ids", json.dumps(controls["found_object_ids"]),
            "--controlled-recovery-task-outcomes", json.dumps(
                controls.get("controlled_recovery_task_outcomes", {})
            ),
        ]
        if scene_config:
            command += ["--scene-config", str(scene_config)]
        else:
            command += ["--no-scene-config"]
        if episode["category"] == "ordered_recovery":
            command += [
                "--semantic-recovery", "--max-recovery-hypotheses", "3",
                "--max-recovery-visits", "6", "--max-viewpoints-per-location", "2",
                "--recovery-budget-cny", os.environ.get("PRE_MAP_VLN_QWEN_BUDGET_CNY", "20"),
            ]
        print(f"[{index}/{len(items)}] RUN {episode_id} {episode['category']}", flush=True)
        episode_started = time.monotonic()
        completed = subprocess.run(command, cwd=ROOT, check=False)
        if result_path.exists():
            trace = load_json(result_path)
            status = trace.get("status", "unknown")
            path_m = float(trace.get("path_length_m", 0.0))
        else:
            status, path_m = f"process_exit_{completed.returncode}", 0.0
        record = {
            "episode_id": episode_id, "scene_id": scene_id,
            "category": episode["category"], "status": status,
            "return_code": completed.returncode, "path_length_m": path_m,
            "elapsed_wall_s": time.monotonic() - episode_started,
            "episode_sha256": digest, "verification_mode": args.verification_mode,
        }
        atomic_json(output / "benchmark_status.json", record)
        results.append(record)
    report = {
        "format": "pre_map_vln.benchmark_execution_batch.v1",
        "requested_count": len(items), "elapsed_wall_s": time.monotonic() - started,
        "results": results,
    }
    atomic_json(args.root / "runs/execution_batch_report.json", report)


if __name__ == "__main__":
    main()
