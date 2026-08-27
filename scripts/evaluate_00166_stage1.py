#!/usr/bin/env python3
"""Evaluate one 00166 stage-1 run against Habitat navmesh truth.

The planner is deliberately not given any of the truth computed here.  This
script only consumes the finished episode/Bag exports and the official scene
assets, then writes auditable coverage, map-state, trajectory and mesh-proxy
metrics.  The two floors are reported separately because a single fixed
camera height must not be counted as 3-D coverage.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from collections import deque
from pathlib import Path

import habitat_sim
import numpy as np


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
FALCON_SENSOR_ORIGIN = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)


def load_pcd(path: Path) -> np.ndarray:
    if not path.is_file():
        return np.empty((0, 3), dtype=np.float64)
    values = np.loadtxt(path, skiprows=11, dtype=np.float64)
    if values.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    values = np.atleast_2d(values)
    return values[:, :3]


def read_trajectory(path: Path) -> np.ndarray:
    if not path.is_file():
        return np.empty((0, 8), dtype=np.float64)
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append([
                float(row[key])
                for key in ("time", "x", "y", "z", "qx", "qy", "qz", "qw")
            ])
    return np.asarray(rows, dtype=np.float64)


def read_episode_positions(episode: Path) -> np.ndarray:
    positions = []
    for path in sorted(episode.glob("frame_*.npz")):
        with np.load(path) as frame:
            positions.append(np.asarray(frame["position"], dtype=np.float64))
    return np.asarray(positions, dtype=np.float64)


def clean_log(path: Path) -> str:
    return re.sub(r"\x1b\[[0-9;]*m", "", path.read_text(errors="replace"))


def log_metrics(path: Path) -> dict:
    text = clean_log(path) if path.is_file() else ""

    def count(pattern: str) -> int:
        return len(re.findall(pattern, text, flags=re.IGNORECASE))

    frontier_rows = [
        (int(active), int(dormant))
        for active, dormant in re.findall(
            r"Frontier number:\s*(\d+),\s*dormant frontier number:\s*(\d+)", text
        )
    ]
    coverage = [float(value) for value in re.findall(r"Coverage:\s*([0-9]+(?:\.[0-9]+)?)", text)]
    targets = [
        np.asarray([float(x), float(y), float(z)], dtype=np.float64)
        for x, y, z in re.findall(
            r"Next pos:\s*(-?[0-9.]+),\s*(-?[0-9.]+),\s*(-?[0-9.]+)", text
        )
    ]
    aba_events = 0
    repeated_targets = 0
    for index in range(2, len(targets)):
        if np.linalg.norm(targets[index] - targets[index - 2]) <= 0.25:
            if np.linalg.norm(targets[index] - targets[index - 1]) > 0.35:
                aba_events += 1
        if np.linalg.norm(targets[index] - targets[index - 1]) <= 0.15:
            repeated_targets += 1

    finish_reasons = re.findall(r"Finish exploration:\s*([^\r\n\x1b]+)", text)
    transition_to_finish = count(r"Transit state from .*? to FINISH")
    return {
        "finish_transition_count": transition_to_finish,
        "finish_reasons": [reason.strip() for reason in finish_reasons],
        "planner_coverage_last": None if not coverage else coverage[-1],
        "planner_coverage_max": None if not coverage else max(coverage),
        "planner_coverage_samples": len(coverage),
        "frontier_cluster_last": None if not frontier_rows else list(frontier_rows[-1]),
        "frontier_cluster_max": None if not frontier_rows else max(row[0] for row in frontier_rows),
        "dormant_cluster_last": None if not frontier_rows else frontier_rows[-1][1],
        "dormant_cluster_max": None if not frontier_rows else max(row[1] for row in frontier_rows),
        "next_target_count": len(targets),
        "target_aba_events": aba_events,
        "repeated_target_events": repeated_targets,
        "predicted_collision_events": count(
            r"Collision detected on the trajectory before publishing"
        ),
        "collision_detection_log_lines": count(r"FastPlannerManager\] Collision detected at"),
        "trajectory_discontinuity_events": count(r"Position trajectory discontinuity detected"),
        "no_viewpoint_events": count(r"No frontier viewpoint in next grid cell"),
        "no_free_subspace_events": count(r"no free subspace"),
        "astar_no_path_events": count(r"planTrajToView: No path"),
        "time_lower_bound_events": count(r"Time lower bound not satified"),
        "all_cells_disconnected_events": count(r"All cells are disconnected"),
    }


def target_layer(
    pathfinder,
    floor_height: float,
    resolution: float,
    lower_h: np.ndarray,
    origin_h: np.ndarray,
    grid_origin: np.ndarray,
    grid_shape: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray]:
    """Return a grid mask and transformed FALCON XY centers for one floor."""
    topdown = np.asarray(pathfinder.get_topdown_view(resolution, float(floor_height)), dtype=bool)
    rows, cols = np.nonzero(topdown)
    habitat_x = lower_h[0] + (cols.astype(np.float64) + 0.5) * resolution
    habitat_z = lower_h[2] + (rows.astype(np.float64) + 0.5) * resolution
    falcon_xy = np.column_stack((
        -(habitat_z - origin_h[2]),
        -(habitat_x - origin_h[0]),
    ))
    indices = np.floor((falcon_xy - grid_origin) / resolution).astype(np.int64)
    in_bounds = (
        (indices[:, 0] >= 0) & (indices[:, 1] >= 0)
        & (indices[:, 0] < grid_shape[1]) & (indices[:, 1] < grid_shape[0])
    )
    indices = indices[in_bounds]
    falcon_xy = falcon_xy[in_bounds]
    mask = np.zeros(grid_shape, dtype=bool)
    if len(indices):
        mask[indices[:, 1], indices[:, 0]] = True
    return mask, falcon_xy


def connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    visited = np.zeros(mask.shape, dtype=bool)
    components = []
    for y, x in zip(*np.nonzero(mask)):
        if visited[y, x]:
            continue
        queue = deque([(int(y), int(x))])
        visited[y, x] = True
        component = []
        while queue:
            yy, xx = queue.popleft()
            component.append((yy, xx))
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = yy + dy, xx + dx
                if (
                    0 <= ny < mask.shape[0] and 0 <= nx < mask.shape[1]
                    and mask[ny, nx] and not visited[ny, nx]
                ):
                    visited[ny, nx] = True
                    queue.append((ny, nx))
        components.append(component)
    return components


def cell_coordinates(cells, grid_origin: np.ndarray, resolution: float) -> np.ndarray:
    indices = np.asarray([(x, y) for y, x in cells], dtype=np.float64)
    return grid_origin + (indices + 0.5) * resolution


def layer_metrics(
    name: str,
    target: np.ndarray,
    grid: np.ndarray,
    grid_origin: np.ndarray,
    resolution: float,
) -> tuple[dict, np.ndarray]:
    values = grid[target]
    total = len(values)
    free = int(np.count_nonzero(values == 0))
    occupied = int(np.count_nonzero(values == 100))
    unknown = int(np.count_nonzero(values < 0))
    known = total - unknown
    remaining = target & (grid < 0)
    conflicts = target & (grid == 100)
    components = []
    for component in sorted(connected_components(remaining), key=len, reverse=True)[:20]:
        points = cell_coordinates(component, grid_origin, resolution)
        components.append({
            "cells": len(component),
            "area_m2": len(component) * resolution * resolution,
            "centroid_xy_m": points.mean(axis=0).round(4).tolist(),
            "bbox_min_xy_m": points.min(axis=0).round(4).tolist(),
            "bbox_max_xy_m": points.max(axis=0).round(4).tolist(),
        })
    result = {
        "name": name,
        "target_cells": total,
        "truth_raster_area_m2": total * resolution * resolution,
        "target_in_observed_grid": total,
        "observed_free_cells": free,
        "observed_occupied_cells": occupied,
        "observed_unknown_cells": unknown,
        "known_fraction": None if not total else known / total,
        "strict_free_coverage_fraction": None if not total else free / total,
        "unknown_fraction": None if not total else unknown / total,
        "occupied_conflict_fraction": None if not total else occupied / total,
        "remaining_unknown_area_m2": unknown * resolution * resolution,
        "remaining_unknown_components_top20": components,
    }
    return result, remaining


def online_map_layer_metrics(
    target: np.ndarray,
    map_dir: Path,
    grid_origin: np.ndarray,
    resolution: float,
    z_min: float,
    z_max: float,
    expected_sensor_z: float,
    map_resolution: float,
) -> dict:
    if expected_sensor_z < z_min or expected_sensor_z > z_max:
        return {
            "scope": "outside_online_map_z_band",
            "expected_sensor_z_m": expected_sensor_z,
            "map_z_band_m": [z_min, z_max],
            "target_cells": int(np.count_nonzero(target)),
            "online_free_cells": None,
            "online_occupied_cells": None,
            "online_unknown_cells": None,
            "known_fraction": None,
            "strict_free_coverage_fraction": None,
            "unknown_fraction": None,
            "point_counts_in_band": None,
            "note": "A 2-D ground map must not be used to claim coverage of this floor.",
        }
    ys, xs = np.nonzero(target)
    target_xy = grid_origin + np.column_stack((xs + 0.5, ys + 0.5)) * resolution
    target_keys = {
        (int(np.floor(x / map_resolution)), int(np.floor(y / map_resolution)))
        for x, y in target_xy
    }
    sets = {}
    for name in ("free", "occupied", "unknown"):
        points = load_pcd(map_dir / f"map_{name}.pcd")
        points = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
        indices = np.floor(points[:, :2] / map_resolution).astype(np.int64)
        sets[name] = {(int(x), int(y)) for x, y in indices}
    free = len(target_keys & sets["free"])
    occupied = len((target_keys - sets["free"]) & sets["occupied"])
    unknown = len(target_keys - sets["free"] - sets["occupied"])
    return {
        "map_z_band_m": [z_min, z_max],
        "expected_sensor_z_m": expected_sensor_z,
        "online_map_resolution_m": map_resolution,
        "target_map_cells": len(target_keys),
        "online_free_cells": free,
        "online_occupied_cells": occupied,
        "online_unknown_cells": unknown,
        "known_fraction": None if not target_keys else (free + occupied) / len(target_keys),
        "strict_free_coverage_fraction": None if not target_keys else free / len(target_keys),
        "unknown_fraction": None if not target_keys else unknown / len(target_keys),
        "point_counts_in_band": {name: len(sets[name]) for name in sets},
    }


def trajectory_stats(trajectory: np.ndarray) -> dict:
    if len(trajectory) == 0:
        return {"samples": 0}
    positions = trajectory[:, 1:4]
    deltas = np.diff(positions, axis=0)
    if len(trajectory) > 1:
        dt = np.diff(trajectory[:, 0])
        valid_dt = np.where(dt > 1e-6, dt, np.nan)
        speeds = np.linalg.norm(deltas, axis=1) / valid_dt
        speeds = speeds[np.isfinite(speeds)]
    else:
        speeds = np.empty(0, dtype=np.float64)
    return {
        "samples": len(trajectory),
        "start_time": float(trajectory[0, 0]),
        "end_time": float(trajectory[-1, 0]),
        "path_length_m": float(np.linalg.norm(deltas, axis=1).sum()),
        "position_min_xyz_m": positions.min(axis=0).round(5).tolist(),
        "position_max_xyz_m": positions.max(axis=0).round(5).tolist(),
        "position_mean_xyz_m": positions.mean(axis=0).round(5).tolist(),
        "position_std_xyz_m": positions.std(axis=0).round(5).tolist(),
        "stationary_fraction_speed_below_0.02_mps": (
            None if not len(speeds) else float(np.mean(speeds < 0.02))
        ),
    }


def mesh_and_navmesh_trajectory_metrics(
    sim,
    trajectory: np.ndarray,
    initial_sensor_h: np.ndarray,
) -> dict:
    if len(trajectory) == 0:
        return {"samples": 0}
    step = max(1, len(trajectory) // 400)
    positions_f = trajectory[::step, 1:4]
    body_positions_h = np.asarray([
        initial_sensor_h + S_HABITAT_TO_FALCON.T @ (position - FALCON_SENSOR_ORIGIN)
        - np.asarray([0.0, 1.0, 0.0])
        for position in positions_f
    ], dtype=np.float64)
    directions = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                direction = np.asarray([dx, dy, dz], dtype=np.float32)
                directions.append(direction / np.linalg.norm(direction))

    distances = []
    navigable = []
    for position in body_positions_h:
        navigable.append(bool(sim.pathfinder.is_navigable(position.astype(np.float32))))
        for direction in directions:
            result = sim.cast_ray(
                habitat_sim.geo.Ray(
                    position.astype(np.float32), direction.astype(np.float32)
                )
            )
            if result.has_hits():
                distances.extend(float(hit.ray_distance) for hit in result.hits)
    distances = np.asarray(distances, dtype=np.float64)
    return {
        "samples": len(body_positions_h),
        "body_habitat_y_min_max_m": [
            float(body_positions_h[:, 1].min()), float(body_positions_h[:, 1].max())
        ],
        "body_not_navigable_samples": int(np.count_nonzero(~np.asarray(navigable, dtype=bool))),
        "body_navigable_fraction": float(np.mean(navigable)),
        "mesh_ray_hit_count": len(distances),
        "mesh_sampled_ray_min_distance_m": None if not len(distances) else float(distances.min()),
        "mesh_sampled_ray_p05_distance_m": None if not len(distances) else float(np.percentile(distances, 5)),
        "mesh_distance_method": (
            "minimum positive distance over 26 body-center scene-mesh rays; "
            "an auditable proximity proxy, not a signed distance field"
        ),
    }


def spatial_tiles(target: np.ndarray, grid: np.ndarray, origin: np.ndarray, resolution: float) -> list[dict]:
    ys, xs = np.nonzero(target)
    if not len(xs):
        return []
    fx = origin[0] + (xs + 0.5) * resolution
    fy = origin[1] + (ys + 0.5) * resolution
    result = []
    for iy in range(3):
        for ix in range(3):
            x0, x1 = np.quantile(fx, [ix / 3.0, (ix + 1) / 3.0])
            y0, y1 = np.quantile(fy, [iy / 3.0, (iy + 1) / 3.0])
            selected = target.copy()
            selected &= (
                (np.indices(target.shape)[1] >= np.floor((x0 - origin[0]) / resolution))
                & (np.indices(target.shape)[1] <= np.ceil((x1 - origin[0]) / resolution))
                & (np.indices(target.shape)[0] >= np.floor((y0 - origin[1]) / resolution))
                & (np.indices(target.shape)[0] <= np.ceil((y1 - origin[1]) / resolution))
            )
            values = grid[selected]
            result.append({
                "tile_x": ix,
                "tile_y": iy,
                "target_cells": int(len(values)),
                "strict_free_fraction": None if not len(values) else float(np.mean(values == 0)),
                "known_fraction": None if not len(values) else float(np.mean(values >= 0)),
                "unknown_cells": int(np.count_nonzero(values < 0)),
            })
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--survey", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    episode = args.run_dir / "episode"
    grid_prefix = args.run_dir / "observed_ground"
    grid = np.load(grid_prefix.with_suffix(".npy"))
    grid_meta = json.loads(grid_prefix.with_suffix(".json").read_text(encoding="utf-8"))
    grid_origin = np.asarray(grid_meta["origin_xy_m"], dtype=np.float64)
    resolution = float(grid_meta["resolution_m"])
    manifest = json.loads((episode / "manifest.json").read_text(encoding="utf-8"))
    survey = json.loads(args.survey.read_text(encoding="utf-8"))
    scene_survey = next(
        item for item in survey.get("scenes", [])
        if item.get("scene_id") == "00166-RaYrxWt5pR1"
    )
    origin_h = np.asarray(manifest["initial_agent_habitat_xyz"], dtype=np.float64)
    initial_sensor_h = np.asarray(manifest["initial_sensor_habitat_xyz"], dtype=np.float64)
    trajectory = read_trajectory(args.run_dir / "bag_export" / "trajectory.csv")
    episode_positions = read_episode_positions(episode)
    planner = log_metrics(args.run_dir / "run.log")
    bag_summary_path = args.run_dir / "bag_export" / "summary.json"
    bag_summary = json.loads(bag_summary_path.read_text(encoding="utf-8")) if bag_summary_path.is_file() else {}
    run_result_path = args.run_dir / "run_result.json"
    run_result = json.loads(run_result_path.read_text(encoding="utf-8")) if run_result_path.is_file() else {}
    spec = {}
    spec_path = args.run_dir / "spec.txt"
    if spec_path.is_file():
        for line in spec_path.read_text(encoding="utf-8").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                spec[key] = value

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(args.scene)
    sim_cfg.scene_dataset_config_file = str(args.scene_config)
    sim_cfg.create_renderer = False
    sim_cfg.enable_physics = True
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    floors = []
    target_masks = []
    mesh_metrics = {}
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("00166 Habitat navmesh did not load")
        lower_h, upper_h = (
            np.asarray(value, dtype=np.float64) for value in sim.pathfinder.get_bounds()
        )
        for cluster in scene_survey.get("floor_clusters", []):
            floor_height = float(cluster["height_m"])
            target, _ = target_layer(
                sim.pathfinder, floor_height, resolution, lower_h, origin_h,
                grid_origin, grid.shape,
            )
            target_masks.append(target.copy())
            observed, remaining = layer_metrics(
                "main" if not floors else f"floor_{len(floors)}",
                target, grid, grid_origin, resolution,
            )
            observed["habitat_floor_height_m"] = floor_height
            observed["survey_sample_fraction"] = cluster.get("fraction")
            observed["survey_sample_count"] = cluster.get("samples")
            observed["online_falcon_map"] = online_map_layer_metrics(
                target, args.run_dir / "bag_export", grid_origin, resolution, 0.65, 1.35,
                1.0 + floor_height - float(scene_survey["floor_clusters"][0]["height_m"]), 0.1
            )
            observed["spatial_tiles_3x3"] = spatial_tiles(target, grid, grid_origin, resolution)
            floors.append(observed)
        mesh_metrics = mesh_and_navmesh_trajectory_metrics(sim, trajectory, initial_sensor_h)
        navmesh_area = float(sim.pathfinder.navigable_area)
        bounds = {"min_xyz_m": lower_h.tolist(), "max_xyz_m": upper_h.tolist()}

    if len(floors) >= 2:
        floors[0]["name"] = "main_floor"
        floors[1]["name"] = "upper_floor"

    episode_z = episode_positions[:, 2] if len(episode_positions) else np.empty(0)
    truth_main = floors[0] if floors else {}
    regions = scene_survey.get("regions", [])
    usable_regions = [
        region for region in regions
        if region.get("label") not in (None, "", "unknown")
        and any(float(value) != 0.0 for value in region.get("center_xyz_m", []))
    ]
    result = {
        "format": "pre_map_vln.00166_stage1_truth_metrics.v1",
        "scene": "00166-RaYrxWt5pR1",
        "scene_path": str(args.scene),
        "scene_config": str(args.scene_config),
        "scope": {
            "planner_map_name": spec.get("map_name", "hm3d_00166"),
            "map_dimension_metadata": 2,
            "exploration_scope": "main Habitat floor camera-height band",
            "not_a_complete_3d_exploration": True,
        },
        "truth": {
            "survey_navmesh_area_m2": scene_survey.get("navmesh_area_m2"),
            "habitat_pathfinder_navigable_area_m2": navmesh_area,
            "habitat_bounds": bounds,
            "survey_floor_clusters": scene_survey.get("floor_clusters", []),
            "survey_region_count": scene_survey.get("region_count"),
            "survey_destination_room_count": scene_survey.get("destination_room_count"),
            "usable_labeled_region_count": len(usable_regions),
            "region_note": (
                "00166 survey contains region IDs but labels and centers are unknown/zero; "
                "coverage is therefore reported by floor and spatial tiles, not named rooms."
            ),
        },
        "floors": floors,
        "planner_diagnostics": planner,
        "bag_diagnostics": bag_summary,
        "run_result": run_result,
        "trajectory": {
            "bag_sensor_pose": trajectory_stats(trajectory),
            "episode_recorded_sensor_z_min_max_m": (
                None if not len(episode_z) else [float(episode_z.min()), float(episode_z.max())]
            ),
            "episode_recorded_sensor_z_mean_std_m": (
                None if not len(episode_z) else [float(episode_z.mean()), float(episode_z.std())]
            ),
        },
        "scene_mesh_and_navigation": mesh_metrics,
        "truth_completion_rule": {
            "main_floor_known_fraction_at_least": 0.90,
            "main_floor_unknown_fraction_at_most": 0.10,
            "requires_zero_predicted_collision_events": True,
            "requires_zero_target_aba_events": True,
            "note": "Thresholds are an explicit audit rule; raw fractions remain authoritative.",
            "coverage_acceptance_rule": {
                "main_floor_strict_free_coverage_at_least": 0.90,
                "target_aba_events_are_non_blocking": True,
                "does_not_claim_complete_3d": True,
            },
        },
    }
    result["verdict"] = {
        "entered_legal_finish": run_result.get("termination") == "complete"
        and planner["finish_transition_count"] > 0,
        "main_floor_truth_coverage_sufficient": bool(
            truth_main.get("known_fraction", 0.0) is not None
            and truth_main.get("known_fraction", 0.0) >= 0.90
            and truth_main.get("unknown_fraction", 1.0) <= 0.10
        ),
        "planner_execution_clean": (
            planner["predicted_collision_events"] == 0
            and planner["target_aba_events"] == 0
            and planner["astar_no_path_events"] == 0
        ),
        "complete_by_truth": bool(
            run_result.get("termination") == "complete"
            and planner["finish_transition_count"] > 0
            and truth_main.get("known_fraction", 0.0) is not None
            and truth_main.get("known_fraction", 0.0) >= 0.90
            and truth_main.get("unknown_fraction", 1.0) <= 0.10
            and planner["predicted_collision_events"] == 0
            and planner["target_aba_events"] == 0
        ),
        "accepted_single_floor_by_coverage": bool(
            run_result.get("termination") == "complete"
            and truth_main.get("strict_free_coverage_fraction", 0.0) is not None
            and truth_main.get("strict_free_coverage_fraction", 0.0) >= 0.90
        ),
        "coverage_acceptance_note": (
            "Accepted as a usable single-floor stage-1 map under the requested >=90% "
            "Habitat-truth coverage rule; ABA is reported but non-blocking. "
            "Predicted collision events remain a safety advisory."
        ),
        "stage2_input_recommendation": "usable only as a diagnosed single-floor map; not as complete 3-D input",
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    np.savez_compressed(
        args.output.with_suffix(".npz"),
        observed_grid=grid,
        main_floor_target=(
            target_masks[0] if target_masks else np.zeros_like(grid, dtype=bool)
        ),
        upper_floor_target=(
            target_masks[1] if len(target_masks) > 1 else np.zeros_like(grid, dtype=bool)
        ),
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
