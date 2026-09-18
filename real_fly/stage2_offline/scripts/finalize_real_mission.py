#!/usr/bin/env python3
"""Collision-check and freeze an offline mission for real-machine preview.

This program is deliberately ROS-free.  It never starts hardware and never
publishes a command.  The validated B-spline is exported as a continuous
full_smooth route; the senior full_smooth_mission node is the runtime MINCO
executor for that route.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from stage2.bspline_3d import (  # noqa: E402
    BsplineSettings,
    anchor_astar_path,
    plan_collision_checked_bspline,
)
from stage2.io_utils import atomic_json, load_json  # noqa: E402
from stage2.planning_contract import PlannerProfile  # noqa: E402
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def _xyz(values: Any, label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain three finite values")
    return result


def _execution_limitations(task_graph: dict[str, Any]) -> list[str]:
    limitations = []
    if task_graph.get("conditional_rules"):
        limitations.append("conditional_outcome_branch_requires_online_perception_executor")
    if any(
        task.get("intent", {}).get("not_found_policy") == "semantic_recovery"
        or task.get("search_policy", {}).get("on_exhaustion") == "qwen_semantic_recovery"
        for task in task_graph.get("tasks", [])
    ):
        limitations.append("semantic_recovery_requires_online_perception_executor")
    return limitations


def _execution_waypoints(
    points: list[list[float]], voxel_map: VoxelMap3D, maximum_leg_m: float,
) -> list[list[float]]:
    """Keep the curve shape without turning every B-spline sample into a goal."""
    if len(points) <= 1:
        return list(points)
    result = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        selected = anchor + 1
        for candidate in range(anchor + 1, len(points)):
            if math.dist(points[anchor], points[candidate]) > maximum_leg_m + 1e-9:
                break
            if voxel_map.line_is_valid(points[anchor], points[candidate]):
                selected = candidate
        result.append(points[selected])
        anchor = selected
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--task-graph", type=Path, required=True)
    parser.add_argument("--voxel-snapshot", type=Path, required=True)
    parser.add_argument("--planning-config", type=Path, required=True)
    parser.add_argument(
        "--start-source", choices=("planning_start_preview", "explicit_approved_pose"),
        required=True,
    )
    parser.add_argument("--output-mission", type=Path, required=True)
    parser.add_argument("--output-audit", type=Path, required=True)
    args = parser.parse_args()

    for path in (
        args.mission,
        args.task_graph,
        args.voxel_snapshot / "metadata.json",
        args.voxel_snapshot / "voxel_map.npz",
        args.planning_config,
    ):
        if not path.is_file() or path.stat().st_size == 0:
            raise SystemExit(f"missing or empty input: {path}")
    for path in (args.output_mission, args.output_audit):
        if path.exists():
            raise SystemExit(f"refusing to overwrite: {path}")

    mission = copy.deepcopy(load_json(args.mission))
    task_graph = load_json(args.task_graph)
    if mission.get("format") != "pre_map_vln.mission_plan.v1":
        raise SystemExit(f"unsupported mission format: {mission.get('format')}")
    if task_graph.get("format") != "pre_map_vln.task_graph.v1":
        raise SystemExit(f"unsupported task graph format: {task_graph.get('format')}")

    profile = PlannerProfile.load(args.planning_config)
    voxel_map = VoxelMap3D.load(args.voxel_snapshot, profile)
    settings = BsplineSettings.load(args.planning_config)
    waypoint_leg_m = min(0.70, settings.conditional_prefix_length_m)
    identity = voxel_map.identity
    for key, expected in (
        ("map_epoch_uuid", identity.map_epoch_uuid),
        ("geometry_map_version", identity.geometry_map_version),
        ("planner_profile_hash", identity.planner_profile_hash),
    ):
        if mission.get(key) != expected:
            raise SystemExit(f"mission/voxel identity mismatch: {key}")

    visits = mission.get("visits", [])
    segments = mission.get("segments", [])
    if not visits or len(visits) != len(segments):
        raise SystemExit("mission must contain one non-empty segment per visit")
    start = [float(value) for value in mission.get("start_xyz_yaw", [])]
    if len(start) != 4 or not all(math.isfinite(value) for value in start):
        raise SystemExit("mission start_xyz_yaw must contain four finite values")
    if not voxel_map.is_state_valid(start[:3]):
        raise SystemExit("mission start is not in inflated FREE space")

    known_tasks = {task["id"] for task in task_graph.get("tasks", [])}
    previous = start[:3]
    minimum_clearance = math.inf
    modes: list[str] = []
    for index, (visit, segment) in enumerate(zip(visits, segments)):
        task_id = str(visit.get("task_id", ""))
        if task_id not in known_tasks:
            raise SystemExit(f"visit {index} references unknown task: {task_id}")
        pose = visit.get("pose", {})
        goal = _xyz([pose.get("x"), pose.get("y"), pose.get("z")], f"visit {index} pose")
        astar_points = [
            _xyz(point, f"segment {index} A* point")
            for point in segment.get("points_xyz_m", [])
        ]
        if not astar_points:
            raise SystemExit(f"segment {index} has no 3-D A* points")

        if math.dist(previous, goal) <= 1e-7:
            if not voxel_map.is_state_valid(goal):
                raise SystemExit(f"visit {index} same-position goal is not collision-free")
            trajectory = [goal]
            mode, degree, smoothing, pieces = "same_position_yaw_only", 0, 0.0, 1
            clearance = voxel_map.clearance(goal)
        else:
            try:
                anchored = anchor_astar_path(astar_points, previous, goal, voxel_map)
                fitted = plan_collision_checked_bspline(anchored, voxel_map, settings)
            except (RuntimeError, TypeError, ValueError) as error:
                raise SystemExit(f"segment {index} B-spline validation failed: {error}") from error
            trajectory = fitted.points_xyz_m
            mode, degree = fitted.mode, fitted.degree
            smoothing, pieces = fitted.smoothing_m, fitted.piece_count
            clearance = fitted.minimum_clearance_m
        if not all(voxel_map.is_state_valid(point) for point in trajectory):
            raise SystemExit(f"segment {index} trajectory leaves inflated FREE space")
        if not all(
            voxel_map.line_is_valid(first, second)
            for first, second in zip(trajectory, trajectory[1:])
        ):
            raise SystemExit(f"segment {index} trajectory failed continuous collision recheck")

        segment["validated_trajectory"] = {
            "points_xyz_m": trajectory,
            "execution_waypoints_xyz_m": _execution_waypoints(
                trajectory, voxel_map, waypoint_leg_m,
            ),
            "maximum_waypoint_leg_m": waypoint_leg_m,
            "mode": mode,
            "degree": degree,
            "smoothing_m": smoothing,
            "piece_count": pieces,
            "minimum_clearance_m": clearance,
            "offline_reference_only": True,
            "authoritative_runtime_planner": "full_smooth_mission",
        }
        minimum_clearance = min(minimum_clearance, clearance)
        modes.append(mode)
        previous = goal

    dynamic_limitations = _execution_limitations(task_graph)
    online_limitations = [
        "online_target_verification_not_connected",
        *dynamic_limitations,
    ]
    audit = {
        "format": "pre_map_vln.real_mission_audit.v1",
        "status": "pass",
        "scope": "offline_motion_preview_only",
        "task_count": len(task_graph.get("tasks", [])),
        "planned_visit_count": len(visits),
        "segment_count": len(segments),
        "minimum_clearance_m": minimum_clearance,
        "trajectory_modes": modes,
        "map_identity": {
            "map_epoch_uuid": identity.map_epoch_uuid,
            "geometry_map_version": identity.geometry_map_version,
            "semantic_map_version": identity.semantic_map_version,
            "planner_profile_hash": identity.planner_profile_hash,
        },
        "motion_preview_bundle_eligible": not dynamic_limitations,
        "autonomous_semantic_execution_eligible": False,
        "online_execution_limitations": online_limitations,
        "safety_note": (
            "Passing this audit does not authorize flight. full_smooth_mission must "
            "load the exported route, validate the current start pose, and pass "
            "continuous PositionCommand output through the command mux to PX4Ctrl."
        ),
    }
    mission["offline_trajectory_validation"] = {
        "status": "pass",
        "reference_only": True,
        "minimum_clearance_m": minimum_clearance,
        "authoritative_runtime_planner": "full_smooth_mission",
    }
    mission["real_start_source"] = args.start_source
    mission["start_pose_explicitly_approved"] = args.start_source == "explicit_approved_pose"
    audit["start_source"] = args.start_source
    audit["start_pose_explicitly_approved"] = args.start_source == "explicit_approved_pose"
    atomic_json(args.output_mission, mission)
    atomic_json(args.output_audit, audit)
    print(json.dumps(audit, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
