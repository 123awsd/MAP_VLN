#!/usr/bin/env python3
"""Reconstruct observed multi-floor transition spaces from FALCON outputs.

The online planner is not modified.  Executed trajectories provide high-
confidence seeds; observed FREE voxels estimate the interior; vertically
continuous OCCUPIED evidence estimates side boundaries.  Missing wall
evidence remains explicitly uncertain instead of being completed as truth.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import median_filter
from scipy.spatial import cKDTree

if __package__:
    from scripts.run_multifloor_room_pipeline import infer_levels, read_pcd_xyz
else:
    from run_multifloor_room_pipeline import infer_levels, read_pcd_xyz


def read_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    times = np.asarray([float(row["time"]) for row in rows], dtype=np.float64)
    points = np.asarray(
        [[float(row[key]) for key in ("x", "y", "z")] for row in rows],
        dtype=np.float32,
    )
    finite = np.isfinite(times) & np.isfinite(points).all(axis=1)
    return times[finite], points[finite]


def run_length_encode(labels: np.ndarray) -> list[tuple[int, int, int]]:
    if not len(labels):
        return []
    starts = np.r_[0, np.flatnonzero(labels[1:] != labels[:-1]) + 1]
    ends = np.r_[starts[1:], len(labels)]
    return [(int(labels[start]), int(start), int(end)) for start, end in zip(starts, ends)]


def resample_polyline(points: np.ndarray, spacing: float = 0.20) -> np.ndarray:
    if len(points) < 2:
        return points.copy()
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) >= 0.03]
    points = points[keep]
    if len(points) < 2:
        return points
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    samples = np.arange(0.0, distance[-1], spacing)
    samples = np.r_[samples, distance[-1]]
    return np.column_stack([
        np.interp(samples, distance, points[:, axis]) for axis in range(3)
    ]).astype(np.float32)


def detect_transition_traversals(
    times: np.ndarray,
    trajectory: np.ndarray,
    levels: list[dict],
    stable_band: float = 0.62,
    minimum_stable_seconds: float = 0.80,
) -> list[dict]:
    """Find stable-floor A -> B traversals with debounced floor labels."""
    if len(trajectory) < 10 or len(levels) < 2:
        return []
    flight_z = np.asarray([item["flight_z_m"] for item in levels], dtype=np.float32)
    dt = float(np.median(np.diff(times))) if len(times) > 1 else 0.1
    smooth_samples = max(3, int(round(0.35 / max(dt, 1e-3))))
    if smooth_samples % 2 == 0:
        smooth_samples += 1
    smooth = trajectory.copy()
    for axis in range(3):
        smooth[:, axis] = median_filter(trajectory[:, axis], size=smooth_samples, mode="nearest")

    nearest = np.argmin(abs(smooth[:, 2, None] - flight_z[None, :]), axis=1)
    vote_samples = max(3, int(round(0.70 / max(dt, 1e-3))))
    votes = np.column_stack([
        np.convolve((nearest == floor).astype(np.float32), np.ones(vote_samples), mode="same")
        for floor in range(len(flight_z))
    ])
    labels = np.argmax(votes, axis=1).astype(np.int32)
    minimum_samples = max(3, int(round(minimum_stable_seconds / max(dt, 1e-3))))
    stable_runs = [run for run in run_length_encode(labels) if run[2] - run[1] >= minimum_samples]

    traversals = []
    for source, destination in zip(stable_runs, stable_runs[1:]):
        source_floor, source_start, source_end = source
        target_floor, target_start, target_end = destination
        if source_floor == target_floor or abs(source_floor - target_floor) != 1:
            continue
        source_band = np.flatnonzero(
            abs(smooth[source_start:source_end, 2] - flight_z[source_floor]) <= stable_band
        )
        target_band = np.flatnonzero(
            abs(smooth[target_start:target_end, 2] - flight_z[target_floor]) <= stable_band
        )
        if not len(source_band) or not len(target_band):
            continue
        start = source_start + int(source_band[-1])
        end = target_start + int(target_band[0])
        if end <= start + 2 or times[end] - times[start] > 18.0:
            continue
        lower, upper = sorted((source_floor, target_floor))
        midpoint_z = 0.5 * (flight_z[lower] + flight_z[upper])
        segment_z = smooth[start:end + 1, 2]
        if segment_z.min() > midpoint_z or segment_z.max() < midpoint_z:
            continue
        segment = resample_polyline(smooth[start:end + 1], spacing=0.18)
        if len(segment) < 3:
            continue
        crossing = int(np.argmin(abs(segment[:, 2] - midpoint_z)))
        lower_endpoint = segment[0] if source_floor == lower else segment[-1]
        upper_endpoint = segment[-1] if target_floor == upper else segment[0]
        traversals.append({
            "source_floor": source_floor + 1,
            "target_floor": target_floor + 1,
            "lower_floor": lower + 1,
            "upper_floor": upper + 1,
            "start_time": float(times[start]),
            "end_time": float(times[end]),
            "lower_entrance_xyz": lower_endpoint.tolist(),
            "upper_entrance_xyz": upper_endpoint.tolist(),
            "crossing_xyz": segment[crossing].tolist(),
            "centerline_xyz": segment.tolist(),
            "path_length_m": float(np.linalg.norm(np.diff(segment, axis=0), axis=1).sum()),
        })
    return traversals


def cluster_traversals(traversals: list[dict], radius: float = 2.0) -> list[list[dict]]:
    groups: list[list[dict]] = []
    for traversal in traversals:
        crossing = np.asarray(traversal["crossing_xyz"], dtype=np.float32)
        for group in groups:
            representative = np.asarray(group[0]["crossing_xyz"], dtype=np.float32)
            same_floors = (
                traversal["lower_floor"] == group[0]["lower_floor"]
                and traversal["upper_floor"] == group[0]["upper_floor"]
            )
            if same_floors and np.linalg.norm(crossing[:2] - representative[:2]) <= radius:
                group.append(traversal)
                break
        else:
            groups.append([traversal])
    return groups


def local_sections(
    centerline: np.ndarray,
    occupied: np.ndarray,
    free: np.ndarray,
    maximum_half_width: float = 1.80,
) -> list[dict]:
    """Estimate path-normal corridor sections and wall-side confidence."""
    occupied_tree = cKDTree(occupied)
    free_tree = cKDTree(free)
    raw = []
    for index, center in enumerate(centerline):
        before = centerline[max(0, index - 2), :2]
        after = centerline[min(len(centerline) - 1, index + 2), :2]
        tangent = after - before
        norm = float(np.linalg.norm(tangent))
        if norm < 1e-4:
            tangent = np.asarray([1.0, 0.0], dtype=np.float32)
        else:
            tangent = tangent / norm
        normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float32)
        occupied_near = occupied[occupied_tree.query_ball_point(center, r=2.6)]
        free_near = free[free_tree.query_ball_point(center, r=2.3)]

        def project(points, vertical=0.80, longitudinal=0.30):
            if not len(points):
                return np.empty((0, 2), dtype=np.float32)
            delta = points - center
            long = delta[:, :2] @ tangent
            mask = (abs(long) <= longitudinal) & (abs(delta[:, 2]) <= vertical)
            return np.column_stack((delta[mask, :2] @ normal, delta[mask, 2]))

        occupied_projection = project(occupied_near, vertical=1.10, longitudinal=0.28)
        free_projection = project(free_near, vertical=0.45, longitudinal=0.32)
        wall_offsets = []
        observed_walls = []
        for side in (-1, 1):
            best = None
            observed_wall = False
            if len(occupied_projection):
                lateral = occupied_projection[:, 0]
                edges = np.arange(0.25, maximum_half_width + 0.11, 0.10)
                for lower in edges:
                    cells = occupied_projection[
                        (side * lateral >= lower) & (side * lateral < lower + 0.10)
                    ]
                    if len(cells) >= 3 and np.ptp(cells[:, 1]) >= 0.75:
                        best = float(side * (lower + 0.05))
                        observed_wall = True
                        break
            if best is None and len(free_projection):
                candidates = side * free_projection[:, 0]
                candidates = candidates[(candidates >= 0.0) & (candidates <= maximum_half_width)]
                if len(candidates):
                    best = float(side * min(maximum_half_width, np.percentile(candidates, 92) + 0.10))
            if best is None:
                best = float(side * 0.65)
            wall_offsets.append(best)
            observed_walls.append(observed_wall)
        left_offset, right_offset = min(wall_offsets), max(wall_offsets)
        raw.append({
            "center_xyz": center.tolist(),
            "tangent_xy": tangent.tolist(),
            "normal_xy": normal.tolist(),
            "left_offset_m": left_offset,
            "right_offset_m": right_offset,
            "left_observed_wall": observed_walls[0],
            "right_observed_wall": observed_walls[1],
        })

    widths = np.asarray([item["right_offset_m"] - item["left_offset_m"] for item in raw])
    interior = widths[max(0, len(widths) // 5): max(1, 4 * len(widths) // 5)]
    typical = float(np.median(interior)) if len(interior) else 1.3
    maximum = min(2.8, max(1.0, 1.45 * typical))
    for item in raw:
        left = max(item["left_offset_m"], -0.5 * maximum)
        right = min(item["right_offset_m"], 0.5 * maximum)
        if right - left < 0.65:
            midpoint = 0.5 * (left + right)
            left, right = midpoint - 0.325, midpoint + 0.325
        item["left_offset_m"], item["right_offset_m"] = float(left), float(right)
        center = np.asarray(item["center_xyz"])
        normal = np.asarray(item["normal_xy"])
        item["left_xyz"] = [
            float(center[0] + normal[0] * left), float(center[1] + normal[1] * left), float(center[2])
        ]
        item["right_xyz"] = [
            float(center[0] + normal[0] * right), float(center[1] + normal[1] * right), float(center[2])
        ]
    return raw


def select_corridor_points(points: np.ndarray, sections: list[dict], margin: float) -> np.ndarray:
    if not len(points) or not sections:
        return np.empty((0, 3), dtype=np.float32)
    centers = np.asarray([item["center_xyz"] for item in sections], dtype=np.float32)
    tree = cKDTree(centers)
    distance, nearest = tree.query(points, workers=-1)
    half_width = np.asarray([
        max(abs(item["left_offset_m"]), abs(item["right_offset_m"])) for item in sections
    ], dtype=np.float32)
    z_min, z_max = centers[:, 2].min() - 0.65, centers[:, 2].max() + 0.65
    mask = (
        (distance <= half_width[nearest] + margin)
        & (points[:, 2] >= z_min)
        & (points[:, 2] <= z_max)
    )
    return points[mask]


def floor_opening_score(points: np.ndarray, entrance: np.ndarray, floor_z: float) -> float:
    floor_points = points[abs(points[:, 2] - floor_z) <= 0.14]
    if not len(floor_points):
        return 0.0
    radial = np.linalg.norm(floor_points[:, :2] - entrance[:2], axis=1)
    inner = np.count_nonzero(radial <= 0.65) / (math.pi * 0.65**2)
    outer_area = math.pi * (1.45**2 - 0.85**2)
    outer = np.count_nonzero((radial >= 0.85) & (radial <= 1.45)) / outer_area
    return float(np.clip(1.0 - inner / max(outer, 1e-6), 0.0, 1.0))


def write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
        "TYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA ascii\n"
    )
    with path.open("w", encoding="ascii") as stream:
        stream.write(header)
        if len(points):
            np.savetxt(stream, points, fmt="%.5f %.5f %.5f")


def render_diagnostic(output: Path, occupied: np.ndarray, transitions: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, max(1, len(transitions)), figsize=(7 * max(1, len(transitions)), 7), squeeze=False)
    for axis, transition in zip(axes[0], transitions):
        z0, z1 = transition["z_range_m"]
        background = occupied[(occupied[:, 2] >= z0) & (occupied[:, 2] <= z1)]
        axis.scatter(background[:, 0], background[:, 1], s=.25, c="#b8b8b8", alpha=.20, linewidths=0, rasterized=True)
        for traversal in transition["traversals"]:
            path = np.asarray(traversal["centerline_xyz"])
            axis.plot(path[:, 0], path[:, 1], color="#d000ff", linewidth=2.2)
        sections = transition["boundary_sections"]
        left = np.asarray([item["left_xyz"] for item in sections])
        right = np.asarray([item["right_xyz"] for item in sections])
        axis.plot(left[:, 0], left[:, 1], color="#ff8c00", linewidth=2)
        axis.plot(right[:, 0], right[:, 1], color="#ff8c00", linewidth=2)
        lower = np.asarray(transition["lower_entrance_xyz"])
        upper = np.asarray(transition["upper_entrance_xyz"])
        axis.scatter(lower[0], lower[1], s=75, c="#ffd600", edgecolors="black", label="lower")
        axis.scatter(upper[0], upper[1], s=75, c="#00bcd4", edgecolors="black", label="upper")
        axis.set_title(f'{transition["id"]}: L{transition["connected_floors"][0]}↔L{transition["connected_floors"][1]}')
        axis.set_aspect("equal"); axis.grid(alpha=.15); axis.legend(); axis.set_xlabel("FALCON X (m)"); axis.set_ylabel("FALCON Y (m)")
    if not transitions:
        axes[0, 0].text(.5, .5, "No transition traversal detected", ha="center", va="center")
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def reconstruct(run_dir: Path, output: Path, maximum_floors: int) -> dict:
    levels = infer_levels(run_dir, maximum_floors)
    times, trajectory = read_trajectory(run_dir / "bag_export/trajectory.csv")
    occupied = read_pcd_xyz(run_dir / "bag_export/map_occupied.pcd")
    free = read_pcd_xyz(run_dir / "bag_export/map_free.pcd")
    traversals = detect_transition_traversals(times, trajectory, levels)
    groups = cluster_traversals(traversals)
    transitions = []
    all_transition_points = []
    all_observed_free = []
    for number, group in enumerate(groups, start=1):
        canonical = max(group, key=lambda item: item["path_length_m"])
        centerline = np.asarray(canonical["centerline_xyz"], dtype=np.float32)
        if canonical["source_floor"] == canonical["upper_floor"]:
            centerline = centerline[::-1].copy()
        sections = local_sections(centerline, occupied, free)
        transition_points = select_corridor_points(occupied, sections, margin=0.22)
        observed_free = select_corridor_points(free, sections, margin=0.0)
        all_transition_points.append(transition_points)
        all_observed_free.append(observed_free)
        lower_entrance = np.median(
            np.asarray([item["lower_entrance_xyz"] for item in group]), axis=0
        )
        upper_entrance = np.median(
            np.asarray([item["upper_entrance_xyz"] for item in group]), axis=0
        )
        lower_level = levels[canonical["lower_floor"] - 1]
        upper_level = levels[canonical["upper_floor"] - 1]
        horizontal = float(np.linalg.norm(upper_entrance[:2] - lower_entrance[:2]))
        vertical = float(abs(upper_entrance[2] - lower_entrance[2]))
        wall_sides = sum(
            int(item["left_observed_wall"]) + int(item["right_observed_wall"])
            for item in sections
        )
        observed_fraction = wall_sides / max(1, 2 * len(sections))
        lower_opening = floor_opening_score(occupied, lower_entrance, lower_level["floor_z_m"])
        upper_opening = floor_opening_score(occupied, upper_entrance, upper_level["floor_z_m"])
        confidence = min(
            1.0,
            0.50 + 0.25 * observed_fraction
            + 0.10 * min(1.0, len(group) / 2)
            + 0.15 * 0.5 * (lower_opening + upper_opening),
        )
        transitions.append({
            "id": f"transition_{number}",
            "connected_floors": [canonical["lower_floor"], canonical["upper_floor"]],
            "classification": "stair_candidate" if horizontal >= 0.45 * max(vertical, .1) else "vertical_transition",
            "classification_evidence": "lateral displacement across adjacent persistent flight levels",
            "confidence": float(confidence),
            "boundary_closed": bool(
                observed_fraction >= 0.85 and min(lower_opening, upper_opening) >= 0.15
            ),
            "observed_boundary_fraction": float(observed_fraction),
            "lower_entrance_xyz": lower_entrance.tolist(),
            "upper_entrance_xyz": upper_entrance.tolist(),
            "lower_floor_opening_score": lower_opening,
            "upper_floor_opening_score": upper_opening,
            "z_range_m": [float(centerline[:, 2].min() - .65), float(centerline[:, 2].max() + .65)],
            "boundary_sections": sections,
            "traversals": group,
            "transition_point_count": int(len(transition_points)),
            "observed_free_point_count": int(len(observed_free)),
        })
    output.mkdir(parents=True, exist_ok=True)
    transition_points = np.unique(np.vstack(all_transition_points), axis=0) if all_transition_points else np.empty((0, 3), dtype=np.float32)
    observed_free = np.unique(np.vstack(all_observed_free), axis=0) if all_observed_free else np.empty((0, 3), dtype=np.float32)
    write_ascii_pcd(output / "transition_points.pcd", transition_points)
    write_ascii_pcd(output / "observed_free.pcd", observed_free)
    document = {
        "format": "pre_map_vln.transition_space_reconstruction.v1",
        "uses_hm3d_truth": False,
        "uses_navmesh": False,
        "uses_semantics": False,
        "source_run": str(run_dir),
        "floors": levels,
        "traversal_count": len(traversals),
        "transition_count": len(transitions),
        "transitions": transitions,
    }
    (output / "transitions.json").write_text(json.dumps(document, indent=2) + "\n")
    render_diagnostic(output / "transition_diagnostic.png", occupied, transitions)
    return document


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-floor", type=int, default=0)
    args = parser.parse_args()
    maximum = args.max_floor if args.max_floor > 0 else 16
    document = reconstruct(args.run_dir.resolve(), args.output.resolve(), maximum)
    print(json.dumps({
        "transition_count": document["transition_count"],
        "traversal_count": document["traversal_count"],
        "output": str(args.output.resolve()),
    }, indent=2))


if __name__ == "__main__":
    main()
