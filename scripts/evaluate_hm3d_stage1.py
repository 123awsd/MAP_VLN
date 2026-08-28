#!/usr/bin/env python3
"""Evaluate the seven-scene single-floor stage-1 runs against Habitat truth.

The planner never sees the masks produced here.  Habitat's navmesh is used as
an evaluation reference only; floor clusters are reported independently so a
2-D map cannot accidentally be presented as a multi-floor or 3-D result.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import habitat_sim
import numpy as np

from evaluate_00166_stage1 import (
    layer_metrics,
    log_metrics,
    mesh_and_navmesh_trajectory_metrics,
    online_map_layer_metrics,
    read_episode_positions,
    read_trajectory,
    spatial_tiles,
    target_layer,
    trajectory_stats,
)


SCENES = {
    "00033-oPj9qMxrDEa": "oPj9qMxrDEa",
    "00062-ACZZiU6BXLz": "ACZZiU6BXLz",
    "00087-YY8rqV6L6rf": "YY8rqV6L6rf",
    "00108-oStKKWkQ1id": "oStKKWkQ1id",
    "00150-LcAd9dhvVwh": "LcAd9dhvVwh",
    "00166-RaYrxWt5pR1": "RaYrxWt5pR1",
    "00299-bdp1XNEdvmW": "bdp1XNEdvmW",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scene-id", choices=sorted(SCENES), required=True)
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
        if item.get("scene_id") == args.scene_id
    )
    origin_h = np.asarray(manifest["initial_agent_habitat_xyz"], dtype=np.float64)
    initial_sensor_h = np.asarray(manifest["initial_sensor_habitat_xyz"], dtype=np.float64)
    trajectory = read_trajectory(args.run_dir / "bag_export" / "trajectory.csv")
    episode_positions = read_episode_positions(episode)
    planner = log_metrics(args.run_dir / "run.log")
    bag_summary_path = args.run_dir / "bag_export" / "summary.json"
    bag_summary = (
        json.loads(bag_summary_path.read_text(encoding="utf-8"))
        if bag_summary_path.is_file() else {}
    )
    run_result_path = args.run_dir / "run_result.json"
    run_result = (
        json.loads(run_result_path.read_text(encoding="utf-8"))
        if run_result_path.is_file() else {}
    )
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
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError(f"{args.scene_id} Habitat navmesh did not load")
        lower_h, upper_h = (
            np.asarray(value, dtype=np.float64)
            for value in sim.pathfinder.get_bounds()
        )
        for index, cluster in enumerate(scene_survey.get("floor_clusters", [])):
            floor_height = float(cluster["height_m"])
            target, _ = target_layer(
                sim.pathfinder, floor_height, resolution, lower_h, origin_h,
                grid_origin, grid.shape,
            )
            target_masks.append(target.copy())
            observed, _ = layer_metrics(
                "main_floor" if index == 0 else f"floor_{index}",
                target, grid, grid_origin, resolution,
            )
            observed["habitat_floor_height_m"] = floor_height
            observed["survey_sample_fraction"] = cluster.get("fraction")
            observed["survey_sample_count"] = cluster.get("samples")
            observed["online_falcon_map"] = online_map_layer_metrics(
                target,
                args.run_dir / "bag_export",
                grid_origin,
                resolution,
                0.65,
                1.35,
                1.0 + floor_height - float(scene_survey["floor_clusters"][0]["height_m"]),
                0.1,
            )
            observed["spatial_tiles_3x3"] = spatial_tiles(
                target, grid, grid_origin, resolution
            )
            floors.append(observed)
        mesh_metrics = mesh_and_navmesh_trajectory_metrics(
            sim, trajectory, initial_sensor_h
        )
        navmesh_area = float(sim.pathfinder.navigable_area)
        bounds = {"min_xyz_m": lower_h.tolist(), "max_xyz_m": upper_h.tolist()}

    floor_heights = np.asarray(
        [float(cluster["height_m"]) for cluster in scene_survey.get("floor_clusters", [])],
        dtype=np.float64,
    )
    executed_floor_index = (
        None if not len(floor_heights)
        else int(np.argmin(np.abs(floor_heights - origin_h[1])))
    )
    executed_floor = (
        {} if executed_floor_index is None else floors[executed_floor_index]
    )
    regions = scene_survey.get("regions", [])
    usable_regions = [
        region for region in regions
        if region.get("label") not in (None, "", "unknown")
        and any(float(value) != 0.0 for value in region.get("center_xyz_m", []))
    ]
    episode_z = episode_positions[:, 2] if len(episode_positions) else np.empty(0)
    main_floor = floors[0] if floors else {}
    legal_finish = (
        run_result.get("termination") == "complete"
        and planner["finish_transition_count"] > 0
    )
    result = {
        "format": "pre_map_vln.hm3d_stage1_truth_metrics.v1",
        "scene": args.scene_id,
        "scene_path": str(args.scene),
        "scene_config": str(args.scene_config),
        "scope": {
            "planner_map_name": spec.get("map_name", "hm3d_stage1_seven"),
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
                "The survey region records for this HM3D subset have unknown labels "
                "and zero centers; room-name coverage is not auditable from survey alone."
            ),
            "executed_floor_selection": {
                "initial_agent_habitat_y_m": float(origin_h[1]),
                "nearest_survey_floor_index": executed_floor_index,
                "nearest_survey_floor_height_m": (
                    None if executed_floor_index is None
                    else float(floor_heights[executed_floor_index])
                ),
                "note": "The fixed-height run explores the floor nearest its random Habitat start point; other floors are not counted as covered.",
            },
        },
        "floors": floors,
        "planner_diagnostics": planner,
        "bag_diagnostics": bag_summary,
        "run_result": run_result,
        "trajectory": {
            "bag_sensor_pose": trajectory_stats(trajectory),
            "episode_recorded_sensor_z_min_max_m": (
                None if not len(episode_z)
                else [float(episode_z.min()), float(episode_z.max())]
            ),
            "episode_recorded_sensor_z_mean_std_m": (
                None if not len(episode_z)
                else [float(episode_z.mean()), float(episode_z.std())]
            ),
        },
        "scene_mesh_and_navigation": mesh_metrics,
        "truth_completion_rule": {
            "main_floor_known_fraction_at_least": 0.90,
            "main_floor_unknown_fraction_at_most": 0.10,
            "note": (
                "Main-floor coverage is evaluated against Habitat navmesh raster truth. "
                "Upper floor masks are reported separately and never counted as 2-D completion."
            ),
        },
    }
    result["verdict"] = {
        "entered_legal_finish": legal_finish,
        "main_floor_truth_known_fraction": main_floor.get("known_fraction"),
        "main_floor_truth_strict_free_coverage_fraction": main_floor.get(
            "strict_free_coverage_fraction"
        ),
        "executed_floor_index": executed_floor_index,
        "executed_floor_truth_known_fraction": executed_floor.get("known_fraction"),
        "executed_floor_truth_strict_free_coverage_fraction": executed_floor.get(
            "strict_free_coverage_fraction"
        ),
        "main_floor_truth_coverage_sufficient": bool(
            main_floor.get("known_fraction") is not None
            and main_floor.get("known_fraction") >= 0.90
            and main_floor.get("unknown_fraction", 1.0) <= 0.10
        ),
        "planner_execution_clean": (
            planner["predicted_collision_events"] == 0
            and planner["astar_no_path_events"] == 0
        ),
        "accepted_executed_floor_by_coverage": bool(
            legal_finish
            and executed_floor.get("strict_free_coverage_fraction", 0.0) is not None
            and executed_floor.get("strict_free_coverage_fraction", 0.0) >= 0.90
        ),
        "stage2_input_recommendation": (
            "usable as a diagnosed single-floor map; add floor/room constraints before stage 2"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    np.savez_compressed(
        args.output.with_suffix(".npz"),
        observed_grid=grid,
        floor_targets=np.asarray(target_masks, dtype=bool),
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
