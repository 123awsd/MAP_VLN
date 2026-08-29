#!/usr/bin/env python3
"""Continuously execute a validated multi-floor XYZ mission in Habitat."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import habitat_sim
import numpy as np
from habitat_sim.utils.common import quat_from_angle_axis
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.bspline_3d import (  # noqa: E402
    BsplineSettings,
    anchor_astar_path,
    plan_collision_checked_bspline,
)
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)


def sensor(uuid: str, sensor_type: habitat_sim.SensorType) -> habitat_sim.CameraSensorSpec:
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = uuid
    spec.sensor_type = sensor_type
    spec.resolution = [480, 640]
    spec.position = [0.0, 1.0, 0.0]
    spec.hfov = 90.0
    return spec


def falcon_to_agent_h(initial_agent_h: np.ndarray, pose_f: list[float]) -> np.ndarray:
    sensor_delta_f = np.asarray(pose_f[:3], dtype=np.float64) - np.asarray([0.0, 0.0, 1.0])
    return initial_agent_h + S_HABITAT_TO_FALCON.T @ sensor_delta_f


def interpolate_xyz(points: list[list[float]], step_m: float) -> list[list[float]]:
    if not points:
        return []
    result = [list(points[0])]
    for start, end in zip(points, points[1:]):
        distance = math.dist(start, end)
        steps = max(1, int(math.ceil(distance / step_m)))
        for index in range(1, steps + 1):
            ratio = index / steps
            result.append([
                float(start[axis] + ratio * (end[axis] - start[axis])) for axis in range(3)
            ])
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mission_report", type=Path)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("generated_stage1_config", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--planning-config", type=Path,
        default=ROOT / "config/uav_3d_planning_habitat.yaml",
    )
    parser.add_argument("--step-m", type=float, default=0.12)
    parser.add_argument("--save-every", type=int, default=5)
    args = parser.parse_args()
    profile = PlannerProfile.load(args.planning_config)
    bspline_settings = BsplineSettings.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.snapshot, profile)
    report = load_json(args.mission_report)
    plan = report.get("joint_plan", report)
    generated = load_json(args.generated_stage1_config)
    if report.get("map_epoch_uuid", voxel_map.identity.map_epoch_uuid) != voxel_map.identity.map_epoch_uuid:
        raise RuntimeError("mission plan and voxel snapshot map epochs differ")
    if int(report.get("geometry_map_version", voxel_map.identity.geometry_map_version)) != voxel_map.identity.geometry_map_version:
        raise RuntimeError("mission plan geometry version is stale")
    if report.get("planner_profile_hash", voxel_map.identity.planner_profile_hash) != voxel_map.identity.planner_profile_hash:
        raise RuntimeError("mission plan and execution planner profiles differ")
    for segment in plan.get("segments", []):
        if "points_xyz_m" not in segment:
            raise ValueError("multi-floor Habitat execution requires points_xyz_m for every segment")
        if int(segment["geometry_map_version"]) != voxel_map.identity.geometry_map_version:
            raise RuntimeError("mission segment geometry version is stale")
        if segment.get("planner_profile_hash", voxel_map.identity.planner_profile_hash) != voxel_map.identity.planner_profile_hash:
            raise RuntimeError("mission segment and execution planner profiles differ")

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(Path(generated["scene"]).resolve())
    sim_cfg.scene_dataset_config_file = str(Path(generated["scene_config"]).resolve())
    sim_cfg.enable_physics = True
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [
        sensor("rgb", habitat_sim.SensorType.COLOR),
        sensor("depth", habitat_sim.SensorType.DEPTH),
    ]
    initial_agent_h = np.asarray(generated["initial_agent_habitat_xyz"], dtype=np.float64)
    args.output.mkdir(parents=True, exist_ok=True)
    frames = args.output / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    trajectory, segment_events = [], []
    frame_index = 0
    started = time.monotonic()
    current_yaw = float(report.get("start_xyz_yaw", [0, 0, 1, 0])[3])

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        initial_state = habitat_sim.AgentState()
        initial_state.position = initial_agent_h
        initial_state.rotation = quat_from_angle_axis(0.0, np.asarray([0.0, 1.0, 0.0]))
        agent = sim.initialize_agent(0, initial_state)
        for segment_index, segment in enumerate(plan["segments"]):
            terminal = plan["visits"][segment_index]["pose"]
            actual_goal = [float(terminal[key]) for key in ("x", "y", "z")]
            anchored_astar_points = anchor_astar_path(
                segment["points_xyz_m"], segment["from_xyz_m"], actual_goal, voxel_map,
            )
            bspline = plan_collision_checked_bspline(
                anchored_astar_points, voxel_map, bspline_settings,
            )
            points = interpolate_xyz(bspline.points_xyz_m, args.step_m)
            if not points:
                raise RuntimeError(f"segment {segment_index} contains no path points")
            start_frame = frame_index
            previous = np.asarray(points[0], dtype=float)
            for point in points:
                if not voxel_map.is_state_valid(point):
                    raise RuntimeError(
                        f"segment {segment_index} enters non-FREE voxel at {point}"
                    )
                current = np.asarray(point, dtype=float)
                movement = current - previous
                if np.linalg.norm(movement[:2]) > 1e-6:
                    current_yaw = float(math.atan2(movement[1], movement[0]))
                pose = [float(current[0]), float(current[1]), float(current[2]), current_yaw]
                state = agent.get_state()
                state.position = falcon_to_agent_h(initial_agent_h, pose)
                state.rotation = quat_from_angle_axis(current_yaw, np.asarray([0.0, 1.0, 0.0]))
                agent.set_state(state)
                observations = sim.get_sensor_observations()
                if frame_index % max(1, args.save_every) == 0:
                    Image.fromarray(np.asarray(observations["rgb"])[..., :3].astype(np.uint8)).save(
                        frames / f"frame_{frame_index:06d}.jpg", quality=88
                    )
                trajectory.append(pose)
                previous = current
                frame_index += 1
            current_yaw = float(terminal["yaw"])
            trajectory[-1][3] = current_yaw
            segment_events.append({
                "segment_index": segment_index,
                "task_id": segment["to_task_id"],
                "start_frame": start_frame, "end_frame": frame_index - 1,
                "astar_path_length_m": segment["length_m"],
                "bspline_path_length_m": sum(
                    math.dist(first, second)
                    for first, second in zip(bspline.points_xyz_m, bspline.points_xyz_m[1:])
                ),
                "bspline_mode": bspline.mode,
                "bspline_degree": bspline.degree,
                "bspline_smoothing_m": bspline.smoothing_m,
                "bspline_piece_count": bspline.piece_count,
                "bspline_minimum_clearance_m": bspline.minimum_clearance_m,
                "bspline_collision_checked": True,
                "start_xyz_m": points[0], "end_xyz_m": points[-1],
                "vertical_span_m": max(point[2] for point in points) - min(point[2] for point in points),
            })

    result = {
        "format": "pre_map_vln.stage2_multifloor_habitat_execution.v1",
        "status": "complete",
        "execution_backend": "coarse_3d_astar_collision_checked_bspline_habitat",
        "authoritative_falcon_bspline_validated": False,
        "simulation_bspline_validated": True,
        "planning_mode": "esdf_0_10m_soft_clearance_collision_checked_bspline",
        "note": "Habitat executes a collision-checked simulation B-spline over the locked 3-D A* route. It is not the C++ FALCON B-spline backend.",
        "map_epoch_uuid": voxel_map.identity.map_epoch_uuid,
        "geometry_map_version": voxel_map.identity.geometry_map_version,
        "frame_count": frame_index, "saved_frame_count": len(list(frames.glob("frame_*.jpg"))),
        "elapsed_seconds": time.monotonic() - started,
        "trajectory_xyz_yaw": trajectory, "segments": segment_events,
        "z_min_m": min(item[2] for item in trajectory),
        "z_max_m": max(item[2] for item in trajectory),
        "total_path_length_m": sum(item["bspline_path_length_m"] for item in segment_events),
        "astar_total_path_length_m": sum(item["astar_path_length_m"] for item in segment_events),
        "all_poses_raw_free": True,
        "all_bspline_segments_collision_checked": True,
    }
    clearances = [voxel_map.clearance(item[:3]) for item in trajectory]
    result["clearance_audit"] = {
        "minimum_esdf_m": min(clearances),
        "poses_below_0_20_m": sum(value < 0.20 for value in clearances),
        "poses_below_0_10_m": sum(value + 1e-6 < 0.10 for value in clearances),
        "occupied_or_unknown_pose_count": 0,
        "near_surface_clipping_risk": any(value < 0.20 for value in clearances),
    }
    atomic_json(args.output / "habitat_execution.json", result)
    print(json.dumps({key: value for key, value in result.items() if key != "trajectory_xyz_yaw"}, indent=2))


if __name__ == "__main__":
    main()
