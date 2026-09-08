#!/usr/bin/env python3
"""Build an immutable, auditable real-flight mission bundle.

This script is offline-only. It never imports ROS or publishes commands.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite(values, label):
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise SystemExit(f"non-finite {label}: {values}")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--map", dest="map_pcd", type=Path, required=True)
    parser.add_argument("--voxel-metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-leg-m", type=float, default=1.0,
                        help="maximum spacing between guarded SUPER goals")
    args = parser.parse_args()

    for path in (args.mission, args.map_pcd, args.voxel_metadata):
        if not path.is_file() or path.stat().st_size == 0:
            raise SystemExit(f"missing or empty input: {path}")
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite: {args.output}")
    if not 0.25 <= args.max_leg_m <= 2.0:
        raise SystemExit("--max-leg-m must be in [0.25, 2.0] m")

    mission = json.loads(args.mission.read_text(encoding="utf-8"))
    metadata = json.loads(args.voxel_metadata.read_text(encoding="utf-8"))
    if mission.get("format") != "pre_map_vln.mission_plan.v1":
        raise SystemExit(f"unsupported mission format: {mission.get('format')}")
    if metadata.get("frame_id") != "world":
        raise SystemExit(f"voxel frame must be world, got {metadata.get('frame_id')}")
    identity = metadata.get("identity", {})
    for key in ("map_epoch_uuid", "geometry_map_version", "planner_profile_hash"):
        if mission.get(key) != identity.get(key):
            raise SystemExit(f"mission/voxel identity mismatch: {key}")

    visits = mission.get("visits", [])
    segments = mission.get("segments", [])
    if not visits or len(visits) != len(segments):
        raise SystemExit("mission requires one non-empty segment per visit")
    start = finite(mission.get("start_xyz_yaw", []), "mission start")
    if len(start) != 4:
        raise SystemExit("mission start_xyz_yaw must contain x/y/z/yaw")

    goals = []
    previous = start[:3]
    for index, (visit, segment) in enumerate(zip(visits, segments)):
        validated = segment.get("validated_trajectory") or {}
        source_points = (
            validated.get("execution_waypoints_xyz_m")
            or validated.get("points_xyz_m")
            or segment.get("points_xyz_m", [])
        )
        points = [finite(point, f"segment {index} point") for point in source_points]
        if any(len(point) != 3 for point in points):
            raise SystemExit(f"segment {index} contains a non-XYZ point")
        pose = visit.get("pose", {})
        terminal = finite([pose.get("x"), pose.get("y"), pose.get("z"), pose.get("yaw")],
                          f"visit {index} pose")
        route = list(points)
        if not route or math.dist(route[-1], terminal[:3]) > 1e-6:
            route.append(terminal[:3])
        for point_index, point in enumerate(route):
            distance = math.dist(previous, point)
            terminal_point = point_index == len(route) - 1
            if distance < 0.05:
                if terminal_point:
                    goals.append({
                        "kind": "observation", "xyz": point, "yaw": terminal[3],
                        "visit_index": index, "task_id": visit.get("task_id"),
                        "candidate_id": visit.get("candidate_id"),
                    })
                continue
            if distance > args.max_leg_m + 1e-6:
                subdivisions = int(math.ceil(distance / args.max_leg_m))
                for step in range(1, subdivisions):
                    ratio = step / subdivisions
                    interp = [previous[axis] + ratio * (point[axis] - previous[axis])
                              for axis in range(3)]
                    goals.append({"kind": "transit", "xyz": interp, "yaw": None,
                                  "visit_index": index})
            goals.append({
                "kind": "observation" if terminal_point else "transit",
                "xyz": point,
                "yaw": terminal[3] if terminal_point else None,
                "visit_index": index,
                "task_id": visit.get("task_id") if terminal_point else None,
                "candidate_id": visit.get("candidate_id") if terminal_point else None,
            })
            previous = point

    bundle = {
        "format": "pre_map_vln.real_execution_bundle.v1",
        "status": "preview_only",
        "frame_id": "world",
        "source_mission": str(args.mission.resolve()),
        "source_mission_sha256": sha256(args.mission),
        "map_file_name": args.map_pcd.name,
        "map_sha256": sha256(args.map_pcd),
        "voxel_metadata_sha256": sha256(args.voxel_metadata),
        "map_identity": identity,
        "start_xyz_yaw": start,
        "mission_start_source": mission.get("real_start_source", "unknown"),
        "goal_spacing_limit_m": args.max_leg_m,
        "route_source": "offline_collision_checked_bspline_when_available",
        "goals": goals,
        "safety": {
            "controller": "senior SUPER -> /planning/pos_cmd -> px4ctrl",
            "adapter_publishes_position_command": False,
            "automatic_arming": False,
            "automatic_takeoff": False,
            "automatic_landing": False,
            "start_pose_explicitly_approved": bool(
                mission.get("start_pose_explicitly_approved", False)
            ),
            "autonomous_semantic_execution": False,
            "motion_only_test_requires_explicit_acknowledgement": True,
            "execution_requires_explicit_runtime_confirmation": True,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bundle, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(json.dumps({"output": str(args.output), "map_sha256": bundle["map_sha256"],
                      "goals": len(goals), "visits": len(visits),
                      "status": bundle["status"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
