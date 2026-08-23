#!/usr/bin/env python3
"""Run geometry/task-graph benchmarks on multiple official HM3D example maps."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from pathlib import Path

import habitat_sim
import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.candidate_poses import generate_all_candidates  # noqa: E402
from stage2.grid_map import OccupancyGrid, inflate  # noqa: E402
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.joint_planner import plan_fixed_order_baseline, plan_joint_mission  # noqa: E402
from stage2.mission_executor import simulate_dynamic_execution  # noqa: E402
from stage2.task_graph import normalize_and_validate_task_graph  # noqa: E402


LABELS = ["bed", "lamp", "television", "cabinet", "door", "chair"]


def largest_component(free: np.ndarray) -> np.ndarray:
    visited = np.zeros_like(free, dtype=bool)
    best = []
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
    mask = np.zeros_like(free, dtype=bool)
    if best:
        yy, xx = zip(*best)
        mask[np.asarray(yy), np.asarray(xx)] = True
    return mask


def farthest_points(cells: np.ndarray, count: int) -> list[np.ndarray]:
    center = cells.mean(axis=0)
    first = cells[np.argmin(np.linalg.norm(cells - center, axis=1))]
    selected = [first]
    while len(selected) < count:
        distances = np.min(
            np.stack([np.linalg.norm(cells - point, axis=1) for point in selected]), axis=0
        )
        selected.append(cells[int(np.argmax(distances))])
    return selected


def synthetic_scene_graph(points_xy: list[list[float]], bounds_xy: list[list[float]], scene_name: str):
    objects = []
    for index, (label, point) in enumerate(zip(LABELS, points_xy)):
        objects.append({
            "id": f"navmesh_{index}_{label}",
            "label": label,
            "center_xyz_m": [point[0], point[1], 1.0],
            "size_xyz_m": [0.45, 0.45, 0.7],
            "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
            "probability": 1.0,
            "room_assignment": "synthetic_navmesh_benchmark",
        })
    lower, upper = bounds_xy
    polygon = [[lower[0], lower[1]], [upper[0], lower[1]], [upper[0], upper[1]], [lower[0], upper[1]]]
    return {
        "format": "pre_map_vln.scene_graph.v1",
        "coordinate_frame": "benchmark_navmesh_xy",
        "sources": {"scene": scene_name, "objects": "synthetic anchors for planner generalization only"},
        "summary": {"room_count": 1, "object_count": len(objects), "adjacency_edge_count": 0},
        "rooms": [{
            "id": 1,
            "centroid_xy_m": [(lower[0] + upper[0]) / 2, (lower[1] + upper[1]) / 2],
            "area_m2": (upper[0] - lower[0]) * (upper[1] - lower[1]),
            "polygon_xy_m": polygon,
            "adjacent_room_ids": [],
            "objects": objects,
            "semantic_type": "unknown",
            "semantic_score": 0.0,
        }],
    }


def benchmark_scene(scene_path: Path, task_graph: dict, output_dir: Path, resolution: float):
    simulator_config = habitat_sim.SimulatorConfiguration()
    simulator_config.scene_id = str(scene_path)
    simulator_config.create_renderer = False
    agent_config = habitat_sim.agent.AgentConfiguration()
    with habitat_sim.Simulator(habitat_sim.Configuration(simulator_config, [agent_config])) as sim:
        sim.pathfinder.seed(17)
        height = float(np.asarray(sim.pathfinder.get_random_navigable_point())[1])
        lower, upper = (np.asarray(value, dtype=np.float64) for value in sim.pathfinder.get_bounds())
        topdown = np.asarray(sim.pathfinder.get_topdown_view(resolution, height), dtype=bool)
        component = largest_component(topdown)
        grid_data = np.full(topdown.shape, -1, dtype=np.int8)
        grid_data[component] = 0
        grid = OccupancyGrid(grid_data, [float(lower[0]), float(lower[2])], resolution, inflation_m=0.10)
        reachable = largest_component(~grid.blocked)
        # Favor anchor points with enough nearby free space for terminal-pose rings.
        blocked = ~reachable
        clear = reachable & ~inflate(blocked, max(1, int(round(0.35 / resolution))))
        cells = np.argwhere(clear if np.count_nonzero(clear) >= 50 else component)
        selected = farthest_points(cells, len(LABELS) + 1)
        origin = [float(lower[0]), float(lower[2])]
        def cell_world(cell):
            row, col = cell
            return [origin[0] + (float(col) + 0.5) * resolution, origin[1] + (float(row) + 0.5) * resolution]
        start_xy = cell_world(selected[0])
        object_points = [cell_world(cell) for cell in selected[1:]]
        grid = OccupancyGrid(grid_data, origin, resolution, inflation_m=0.10)
        scene_graph = synthetic_scene_graph(object_points, [origin, [origin[0] + topdown.shape[1] * resolution, origin[1] + topdown.shape[0] * resolution]], scene_path.parent.name)
        candidates = generate_all_candidates(grid, scene_graph, task_graph, max_candidates=6)
        candidates = {
            task_id: [
                candidate for candidate in values
                if grid.astar(start_xy, [candidate["pose"]["x"], candidate["pose"]["y"]]) is not None
            ]
            for task_id, values in candidates.items()
        }
        missing = [key for key, value in candidates.items() if not value]
        if missing:
            raise RuntimeError(f"{scene_path.parent.name}: no candidates for {missing}")
        joint = plan_joint_mission(grid, task_graph, candidates, start_xy + [1.0, 0.0])
        baseline = plan_fixed_order_baseline(grid, task_graph, candidates, start_xy + [1.0, 0.0])
        dynamic = simulate_dynamic_execution(
            grid,
            task_graph,
            candidates,
            start_xy + [1.0, 0.0],
            outcomes={"observe_living_room_tv": "not_found"},
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        np.save(output_dir / "grid.npy", grid_data)
        atomic_json(output_dir / "grid.json", {
            "format": "pre_map_vln.occupancy_grid.v1", "resolution_m": resolution,
            "origin_xy_m": origin, "width": int(grid_data.shape[1]), "height": int(grid_data.shape[0]),
            "free_cells": int(np.count_nonzero(grid_data == 0)), "unknown_cells": int(np.count_nonzero(grid_data == -1)),
            "floor_height_habitat_m": height,
        })
        preview = np.zeros((*grid_data.shape, 3), dtype=np.uint8)
        preview[grid_data == -1] = (55, 55, 55)
        preview[grid_data == 0] = (245, 245, 245)
        Image.fromarray(preview).save(output_dir / "grid.png")
        atomic_json(output_dir / "scene_graph.json", scene_graph)
        atomic_json(output_dir / "candidates.json", {"format": "pre_map_vln.candidates.v1", "by_task": candidates})
        atomic_json(output_dir / "mission_plan.json", joint)
        atomic_json(output_dir / "baseline_plan.json", baseline)
        atomic_json(output_dir / "dynamic_execution.json", dynamic)
        return {
            "scene": scene_path.parent.name,
            "navmesh_area_m2": float(sim.pathfinder.navigable_area),
            "grid_shape": list(grid_data.shape),
            "candidate_count": sum(len(value) for value in candidates.values()),
            "joint_path_length_m": joint["total_path_length_m"],
            "baseline_path_length_m": baseline["total_path_length_m"],
            "improvement_percent": 100.0 * (baseline["total_path_length_m"] - joint["total_path_length_m"]) / max(1e-9, baseline["total_path_length_m"]),
            "dynamic_status": dynamic["status"],
            "dynamic_visits": len(dynamic["executed_visits"]),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-root", type=Path, default=ROOT / "data/scene_datasets/hm3d/example")
    parser.add_argument("--task-graph", type=Path, default=ROOT / "outputs/stage2/hm3d_example/task_graph.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/stage2/multiscene")
    parser.add_argument("--resolution", type=float, default=0.10)
    args = parser.parse_args()
    task_graph = normalize_and_validate_task_graph(load_json(args.task_graph))
    scenes = sorted(args.scene_root.glob("0*/*.basis.glb"))
    if len(scenes) < 3:
        raise RuntimeError(f"expected at least three official HM3D example scenes, found {len(scenes)}")
    results = []
    for scene in scenes:
        result = benchmark_scene(scene.resolve(), task_graph, args.output_dir / scene.parent.name, args.resolution)
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    report = {
        "format": "pre_map_vln.stage2_multiscene.v1",
        "status": "passed" if all(item["dynamic_status"] == "completed" for item in results) else "failed",
        "object_source_note": "Synthetic semantic anchors isolate cross-map geometry/planner generalization; the main 00861 run uses real Boxer and Habitat semantic observations.",
        "scenes": results,
    }
    atomic_json(args.output_dir / "report.json", report)
    print(f"status={report['status']} scenes={len(results)}")


if __name__ == "__main__":
    main()
