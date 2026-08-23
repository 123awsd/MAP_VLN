#!/usr/bin/env python3
"""Run the reproducible multi-scene, multi-seed Stage-2 paper benchmark."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import sys
from collections import defaultdict, deque
from pathlib import Path

import habitat_sim
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.benchmark_suite import build_task_suites, evaluate_method, planner_registry  # noqa: E402
from stage2.candidate_poses import generate_all_candidates  # noqa: E402
from stage2.grid_map import OccupancyGrid  # noqa: E402
from stage2.io_utils import atomic_json  # noqa: E402


LABELS = ["bed", "lamp", "television", "cabinet", "door", "chair"]


def largest_component(free: np.ndarray) -> np.ndarray:
    visited = np.zeros_like(free, dtype=bool)
    best: list[tuple[int, int]] = []
    for row, col in zip(*np.nonzero(free & ~visited)):
        queue = deque([(int(row), int(col))])
        visited[row, col] = True
        component = []
        while queue:
            y, x = queue.popleft()
            component.append((y, x))
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < free.shape[0] and 0 <= nx < free.shape[1] and free[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True
                    queue.append((ny, nx))
        if len(component) > len(best):
            best = component
    result = np.zeros_like(free, dtype=bool)
    if best:
        yy, xx = zip(*best)
        result[np.asarray(yy), np.asarray(xx)] = True
    return result


def seeded_farthest_points(cells: np.ndarray, count: int, rng: random.Random) -> list[np.ndarray]:
    if len(cells) < count:
        raise RuntimeError(f"only {len(cells)} anchor cells available for {count} points")
    selected = [cells[rng.randrange(len(cells))]]
    while len(selected) < count:
        distances = np.min(
            np.stack([np.linalg.norm(cells - point, axis=1) for point in selected]), axis=0
        )
        threshold = np.quantile(distances, 0.98)
        choices = np.flatnonzero(distances >= threshold)
        selected.append(cells[int(choices[rng.randrange(len(choices))])])
    return selected


def scene_graph_from_anchors(
    points_xy: list[list[float]], feasible_angles: list[float],
    bounds_xy: list[list[float]], scene_name: str, seed: int
) -> dict:
    labels = list(LABELS)
    random.Random(seed + 991).shuffle(labels)
    objects = []
    relation_offset = {
        "lamp": 0.0, "television": -math.pi / 2.0,
        "cabinet": math.pi / 2.0, "door": -math.pi,
    }
    for index, (label, point, feasible_angle) in enumerate(zip(labels, points_xy, feasible_angles)):
        # Make each controlled directional task feasible without changing its sampled position.
        yaw = feasible_angle + relation_offset.get(label, 0.0)
        objects.append({
            "id": f"anchor_{seed}_{index}_{label}", "label": label,
            "center_xyz_m": [point[0], point[1], 1.0], "size_xyz_m": [0.45, 0.45, 0.70],
            "orientation_wxyz": [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
            "probability": 1.0, "room_assignment": "synthetic_navmesh_paper_benchmark",
        })
    lower, upper = bounds_xy
    return {
        "format": "pre_map_vln.scene_graph.v1",
        "coordinate_frame": "benchmark_navmesh_xy",
        "sources": {
            "scene": scene_name,
            "objects": "synthetic semantic anchors; real Habitat navmesh geometry",
            "seed": seed,
        },
        "summary": {"room_count": 1, "object_count": len(objects), "adjacency_edge_count": 0},
        "rooms": [{
            "id": 1,
            "centroid_xy_m": [(lower[0] + upper[0]) / 2.0, (lower[1] + upper[1]) / 2.0],
            "area_m2": (upper[0] - lower[0]) * (upper[1] - lower[1]),
            "polygon_xy_m": [lower, [upper[0], lower[1]], upper, [lower[0], upper[1]]],
            "adjacent_room_ids": [], "objects": objects,
            "semantic_type": "synthetic_benchmark_room", "semantic_score": 1.0,
        }],
    }


def build_geometry(scene_path: Path, resolution: float, seed: int):
    simulator_config = habitat_sim.SimulatorConfiguration()
    simulator_config.scene_id = str(scene_path)
    simulator_config.create_renderer = False
    agent_config = habitat_sim.agent.AgentConfiguration()
    with habitat_sim.Simulator(habitat_sim.Configuration(simulator_config, [agent_config])) as sim:
        sim.pathfinder.seed(17)
        lower, upper = (np.asarray(value, dtype=np.float64) for value in sim.pathfinder.get_bounds())
        sampled_heights = {
            round(float(np.asarray(sim.pathfinder.get_random_navigable_point())[1]), 2)
            for _ in range(64)
        }
        floor_options = []
        for sampled_height in sampled_heights:
            view = np.asarray(sim.pathfinder.get_topdown_view(resolution, sampled_height), dtype=bool)
            floor_options.append((int(np.count_nonzero(largest_component(view))), sampled_height, view))
        _, height, topdown = max(floor_options, key=lambda value: (value[0], -abs(value[1])))
        component = largest_component(topdown)
        grid_data = np.full(topdown.shape, -1, dtype=np.int8)
        grid_data[component] = 0
        grid = OccupancyGrid(grid_data, [float(lower[0]), float(lower[2])], resolution, inflation_m=0.10)
        reachable = largest_component(~grid.blocked)
        origin = [float(lower[0]), float(lower[2])]

        def world(cell):
            row, col = cell
            return [origin[0] + (float(col) + 0.5) * resolution, origin[1] + (float(row) + 0.5) * resolution]

        # An anchor is admitted only when at least one candidate on the minimum
        # observation ring is free and visible. This prevents invalid benchmark
        # instances while preserving the real navmesh geometry.
        feasible: list[tuple[np.ndarray, list[float]]] = []
        ring_radius = 0.5 * math.hypot(0.45, 0.45) + 0.8
        for cell in np.argwhere(reachable):
            center = world(cell)
            angles = []
            for sample in range(24):
                angle = 2.0 * math.pi * sample / 24.0
                terminal = [
                    center[0] + ring_radius * math.cos(angle),
                    center[1] + ring_radius * math.sin(angle),
                ]
                if grid.is_free(terminal) and grid.line_is_free(terminal, center, allow_endpoint_cells=2):
                    angles.append(angle)
            if angles:
                feasible.append((cell, angles))
        if len(feasible) < len(LABELS):
            raise RuntimeError(f"{scene_path.parent.name}: only {len(feasible)} feasible semantic anchors")
        feasible_cells = np.stack([item[0] for item in feasible])
        selected_cells = seeded_farthest_points(feasible_cells, len(LABELS), random.Random(seed))
        angle_lookup = {tuple(item[0].tolist()): item[1] for item in feasible}
        anchors = [world(cell) for cell in selected_cells]
        rng = random.Random(seed + 313)
        anchor_angles = [
            angles[rng.randrange(len(angles))]
            for angles in (angle_lookup[tuple(cell.tolist())] for cell in selected_cells)
        ]
        reachable_cells = np.argwhere(reachable)
        anchor_array = np.stack(selected_cells)
        start_distances = np.min(
            np.stack([np.linalg.norm(reachable_cells - point, axis=1) for point in anchor_array]), axis=0
        )
        start_cell = reachable_cells[int(np.argmax(start_distances))]
        start = world(start_cell) + [1.0, 0.0]
        bounds = [origin, [origin[0] + topdown.shape[1] * resolution, origin[1] + topdown.shape[0] * resolution]]
        scene_graph = scene_graph_from_anchors(anchors, anchor_angles, bounds, scene_path.parent.name, seed)
        metadata = {
            "scene": scene_path.parent.name, "seed": seed,
            "navmesh_area_m2": float(sim.pathfinder.navigable_area),
            "grid_shape": list(grid_data.shape), "floor_height_habitat_m": height,
            "feasible_anchor_pool": len(feasible),
        }
    return grid, scene_graph, start, metadata


def trial_cases(suites: dict[str, dict]) -> list[tuple[str, str, dict, dict[str, str]]]:
    cases = []
    for suite_name, graph in suites.items():
        if suite_name == "conditional":
            cases.append((suite_name, "found", graph, {"inspect_tv": "found"}))
            cases.append((suite_name, "not_found", graph, {"inspect_tv": "not_found"}))
        else:
            cases.append((suite_name, "default", graph, {}))
    return cases


def aggregate(records: list[dict], dimensions: tuple[str, ...] = ("method",)) -> list[dict]:
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for record in records:
        groups[tuple(record[key] for key in dimensions)].append(record)
    baseline_by_trial = {
        record["trial_id"]: record["path_length_m"]
        for record in records if record["method"] == "fixed_order_nearest"
    }
    summary = []
    for group_key, values in sorted(groups.items()):
        method = values[0]["method"]
        paths = [value["path_length_m"] for value in values]
        improvements = [
            100.0 * (baseline_by_trial[value["trial_id"]] - value["path_length_m"])
            / max(1e-9, baseline_by_trial[value["trial_id"]])
            for value in values
        ]
        row = {key: value for key, value in zip(dimensions, group_key)}
        row.update({
            "method_group": values[0]["method_group"], "trials": len(values),
            "mission_success_rate": statistics.fmean(value["mission_success"] for value in values),
            "task_completion_rate": statistics.fmean(value["task_completion_rate"] for value in values),
            "constraint_satisfaction_rate": statistics.fmean(value["constraint_satisfaction_rate"] for value in values),
            "condition_satisfaction_rate": statistics.fmean(value["condition_satisfaction"] for value in values),
            "path_length_mean_m": statistics.fmean(paths),
            "path_length_std_m": statistics.pstdev(paths),
            "improvement_vs_fixed_mean_percent": statistics.fmean(improvements),
            "viewpoint_quality_mean": statistics.fmean(value["viewpoint_quality"] for value in values),
            "estimated_time_mean_s": statistics.fmean(value["estimated_time_s"] for value in values),
            "yaw_rotation_mean_rad": statistics.fmean(value["yaw_rotation_rad"] for value in values),
            "planning_time_mean_ms": statistics.fmean(value["planning_time_ms"] for value in values),
        })
        summary.append(row)
    return summary


def markdown_report(report: dict) -> str:
    lines = [
        "# Stage 2 paper benchmark", "",
        f"Scenes: {report['protocol']['scene_count']}; seeds: {report['protocol']['seeds']}; "
        f"task cases: {report['protocol']['task_case_count']}; total trials: {report['protocol']['trial_count']}.", "",
        "Synthetic semantic anchors are used only to isolate planning behavior on real Habitat navmesh geometry.", "",
        "| Method | Group | Success | Constraint | Condition | Path mean±std (m) | Δ vs fixed | View quality | Plan ms |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in report["summary"]:
        lines.append(
            f"| {row['method']} | {row['method_group']} | {100*row['mission_success_rate']:.1f}% | "
            f"{100*row['constraint_satisfaction_rate']:.1f}% | {100*row['condition_satisfaction_rate']:.1f}% | "
            f"{row['path_length_mean_m']:.2f}±{row['path_length_std_m']:.2f} | "
            f"{row['improvement_vs_fixed_mean_percent']:+.2f}% | {row['viewpoint_quality_mean']:.3f} | "
            f"{row['planning_time_mean_ms']:.2f} |"
        )
    lines.extend(["", "This is a geometry/planning benchmark, not an open-vocabulary perception accuracy result.", ""])
    lines.extend([
        "## Proposed method by task suite", "",
        "| Suite | Trials | Path mean±std (m) | Δ vs fixed | Flight time (s) | View quality |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for row in report["summary_by_suite"]:
        if row["method"] != "joint_astar":
            continue
        lines.append(
            f"| {row['suite']} | {row['trials']} | {row['path_length_mean_m']:.2f}±{row['path_length_std_m']:.2f} | "
            f"{row['improvement_vs_fixed_mean_percent']:+.2f}% | {row['estimated_time_mean_s']:.2f} | "
            f"{row['viewpoint_quality_mean']:.3f} |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-root", type=Path, default=ROOT / "data/scene_datasets/hm3d/example")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/stage2/paper_benchmark")
    parser.add_argument("--seeds", type=int, nargs="+", default=[11, 23, 37])
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--max-candidates", type=int, default=6)
    parser.add_argument("--summarize-only", action="store_true")
    args = parser.parse_args()

    existing_report = args.output_dir / "report.json"
    if args.summarize_only:
        previous = json.loads(existing_report.read_text(encoding="utf-8"))
        records = previous["records"]
        previous["summary"] = aggregate(records)
        previous["summary_by_suite"] = aggregate(records, ("method", "suite"))
        previous["summary_by_scene"] = aggregate(records, ("method", "scene"))
        atomic_json(existing_report, previous)
        (args.output_dir / "summary.md").write_text(markdown_report(previous), encoding="utf-8")
        print(json.dumps({"status": previous["status"], "trials": len(records), "summarized": True}))
        return

    scenes = sorted(args.scene_root.glob("0*/*.basis.glb"))
    if len(scenes) < 3:
        raise RuntimeError(f"expected at least three HM3D example scenes, found {len(scenes)}")
    suites = build_task_suites()
    cases = trial_cases(suites)
    methods = planner_registry()
    records = []
    failures = []
    for scene_path in scenes:
        for seed in args.seeds:
            grid, scene_graph, start, geometry = build_geometry(scene_path.resolve(), args.resolution, seed)
            for suite_name, branch, graph, outcomes in cases:
                trial_id = f"{scene_path.parent.name}/seed_{seed}/{suite_name}/{branch}"
                candidates = generate_all_candidates(grid, scene_graph, graph, max_candidates=args.max_candidates)
                missing = [task_id for task_id, values in candidates.items() if not values]
                if missing:
                    failures.append({"trial_id": trial_id, "reason": "no_candidates", "tasks": missing})
                    continue
                for method_name, (method_group, planner) in methods.items():
                    try:
                        # Each method starts with the same empty A* cache. Caching
                        # within its online replans remains part of the method.
                        grid._path_cache.clear()
                        metrics = evaluate_method(
                            method_name, method_group, planner, grid, graph, candidates, start, outcomes
                        )
                        metrics.update({
                            "trial_id": trial_id, "scene": scene_path.parent.name, "seed": seed,
                            "suite": suite_name, "branch": branch,
                            "task_count": graph["summary"]["task_count"],
                            "candidate_count": sum(len(value) for value in candidates.values()),
                            **geometry,
                        })
                        records.append(metrics)
                    except Exception as error:  # keep the complete matrix auditable
                        failures.append({
                            "trial_id": trial_id, "method": method_name,
                            "reason": type(error).__name__, "detail": str(error),
                        })
            print(f"scene={scene_path.parent.name} seed={seed} records={len(records)} failures={len(failures)}", flush=True)

    expected_trials = len(scenes) * len(args.seeds) * len(cases) * len(methods)
    report = {
        "format": "pre_map_vln.stage2_paper_benchmark.v1",
        "status": "passed" if len(records) == expected_trials and not failures else "failed",
        "protocol": {
            "scene_count": len(scenes), "scenes": [path.parent.name for path in scenes],
            "seeds": args.seeds, "task_suites": list(suites), "task_case_count": len(cases),
            "methods": list(methods), "trial_count": len(records), "expected_trial_count": expected_trials,
            "geometry": "real Habitat navmesh", "semantics": "controlled synthetic anchors",
        },
        "summary": aggregate(records),
        "summary_by_suite": aggregate(records, ("method", "suite")),
        "summary_by_scene": aggregate(records, ("method", "scene")),
        "records": records, "failures": failures,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    atomic_json(args.output_dir / "report.json", report)
    (args.output_dir / "summary.md").write_text(markdown_report(report), encoding="utf-8")
    if records:
        scalar_fields = [
            "trial_id", "scene", "seed", "suite", "branch", "method", "method_group",
            "mission_success", "task_completion_rate", "constraint_satisfaction_rate",
            "condition_satisfaction", "path_length_m", "estimated_time_s", "yaw_rotation_rad",
            "planning_time_ms", "viewpoint_quality",
            "replan_count", "task_count", "candidate_count", "navmesh_area_m2",
        ]
        with (args.output_dir / "trials.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=scalar_fields)
            writer.writeheader()
            writer.writerows({key: record[key] for key in scalar_fields} for record in records)
    print(json.dumps({"status": report["status"], "trials": len(records), "failures": len(failures)}, ensure_ascii=False))
    if report["status"] != "passed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
