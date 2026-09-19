#!/usr/bin/env python3
"""Export a validated Stage-2 mission as a full_smooth_mission route.

The route keeps the continuous offline validated trajectory samples as soft
guide points.  The senior full_smooth_mission node consumes this route and
generates the continuous MINCO command stream; it is not converted into a
goal-by-goal PoseStamped mission.
"""

import argparse
import json
import math
from pathlib import Path


def finite_xyz(value, label):
    xyz = [float(axis) for axis in value]
    if len(xyz) != 3 or not all(math.isfinite(axis) for axis in xyz):
        raise SystemExit(f"invalid {label}: {value!r}")
    return xyz


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=0.6)
    parser.add_argument("--dwell", type=float, default=3.0)
    parser.add_argument(
        "--path-source",
        choices=("validated", "astar", "sparse", "sparse_astar"),
        default="validated",
        help=(
            "Guide source for full_smooth_mission (default: validated)."
        ),
    )
    parser.add_argument("--guide-spacing", type=float, default=0.8,
                        help="Approximate spacing for sparse_astar pass points (m).")
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(f"refusing to overwrite: {args.output}")
    if not 0.1 <= args.speed <= 2.0:
        raise SystemExit("--speed must be in [0.1, 2.0] m/s")
    if not 0.1 <= args.dwell <= 30.0:
        raise SystemExit("--dwell must be in [0.1, 30.0] s")
    if not 0.2 <= args.guide_spacing <= 3.0:
        raise SystemExit("--guide-spacing must be in [0.2, 3.0] m")

    mission = json.loads(args.mission.read_text(encoding="utf-8"))
    if mission.get("format") != "pre_map_vln.mission_plan.v1":
        raise SystemExit("unsupported mission format")
    visits = mission.get("visits", [])
    segments = mission.get("segments", [])
    if not visits or len(visits) != len(segments):
        raise SystemExit("mission must contain one segment per visit")

    rows = []
    previous = None
    if args.path_source in ("validated", "astar", "sparse", "sparse_astar"):
        start = mission.get("start_xyz_yaw", [])
        if len(start) != 4 or not all(math.isfinite(float(value)) for value in start):
            raise SystemExit("mission has invalid start_xyz_yaw")
        previous = finite_xyz(start[:3], "mission start")
        # Six-column start anchor preserves approved yaw; pass rows require NaN.
        rows.append((*previous, args.speed, math.degrees(float(start[3])), 0.0, ""))
    for index, (visit, segment) in enumerate(zip(visits, segments)):
        if args.path_source == "validated":
            validated = segment.get("validated_trajectory") or {}
            points = validated.get("points_xyz_m") or segment.get("points_xyz_m") or []
        elif args.path_source == "sparse":
            points = []
        else:
            points = segment.get("points_xyz_m") or []
        if not points and args.path_source != "sparse":
            raise SystemExit(f"segment {index} has no validated execution waypoints")
        points = [finite_xyz(point, f"segment {index} waypoint") for point in points]
        if args.path_source == "sparse_astar" and points:
            sparse_points = [points[0]]
            accumulated = 0.0
            for first, second in zip(points, points[1:]):
                accumulated += math.dist(first, second)
                if accumulated + 1.0e-9 >= args.guide_spacing:
                    sparse_points.append(second)
                    accumulated = 0.0
            if math.dist(sparse_points[-1], points[-1]) > 1.0e-6:
                sparse_points.append(points[-1])
            points = sparse_points
        if points and previous is not None and math.dist(previous, points[0]) <= 1.0e-6:
            points = points[1:]
        pose = visit.get("pose", {})
        terminal = finite_xyz([pose.get("x"), pose.get("y"), pose.get("z")],
                              f"visit {index} pose")
        yaw = float(pose.get("yaw"))
        if not math.isfinite(yaw):
            raise SystemExit(f"visit {index} has invalid yaw")
        if not points or math.dist(points[-1], terminal) > 1.0e-6:
            points.append(terminal)
        for point_index, point in enumerate(points):
            is_terminal = point_index == len(points) - 1
            if is_terminal:
                rows.append((*point, args.speed, math.degrees(yaw), args.dwell, "stop"))
            else:
                rows.append((*point, args.speed, float("nan"), 0.0, "pass"))
            previous = point

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write("# x y z speed_mps yaw_deg dwell_s mode\n")
        for x, y, z, speed, yaw_deg, dwell, mode in rows:
            yaw_text = "nan" if math.isnan(yaw_deg) else f"{yaw_deg:.9f}"
            stream.write(
                f"{x:.9f} {y:.9f} {z:.9f} {speed:.3f} "
                f"{yaw_text} {dwell:.3f} {mode}\n"
            )
    print(json.dumps({"output": str(args.output), "points": len(rows),
                      "stops": len(visits), "speed_mps": args.speed,
                      "dwell_s": args.dwell, "path_source": args.path_source},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
