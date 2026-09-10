#!/usr/bin/env python3
"""Build a geometry-only OccuSG room grid directly from a FAST-LIO PCD."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


def load_binary_pcd_xyz(path: Path) -> tuple[np.ndarray, dict]:
    header: dict[str, list[str]] = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"incomplete PCD header: {path}")
            words = line.decode("ascii", errors="strict").strip().split()
            if not words or words[0].startswith("#"):
                continue
            header[words[0].upper()] = words[1:]
            if words[0].upper() == "DATA":
                offset = stream.tell()
                break
    if header.get("DATA", [""])[0].lower() != "binary":
        raise ValueError("the real-flight room pipeline expects the canonical binary FAST-LIO PCD")
    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if any(value != 1 for value in counts) or not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PCD must contain scalar x/y/z fields")
    codes = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 4): "<i4", ("U", 4): "<u4"}
    dtype = np.dtype([(name, codes[(kind.upper(), size)]) for name, kind, size in zip(fields, types, sizes)])
    points = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
    records = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(points,))
    xyz = np.column_stack([records[name] for name in ("x", "y", "z")]).astype(np.float32)
    return xyz, {"points": points, "fields": fields, "data": "binary"}


def load_trajectory(path: Path) -> np.ndarray:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("trajectory is empty")
    names = rows[0].keys()
    coordinates = [("x", "y", "z"), ("position_x", "position_y", "position_z")]
    selected = next((value for value in coordinates if all(name in names for name in value)), None)
    if selected is None:
        # The real exporter currently writes stamp,x,y,z,... but keep this
        # explicit fallback for older generated runs.
        values = np.loadtxt(path, delimiter=",", skiprows=1)
        return values[:, 1:4].astype(np.float32)
    return np.asarray([[float(row[name]) for name in selected] for row in rows], dtype=np.float32)


def infer_floor_z(points: np.ndarray, trajectory: np.ndarray, bin_size: float = 0.05) -> tuple[float, dict]:
    limit = float(np.median(trajectory[:, 2]) - 0.35)
    candidates = points[:, 2]
    candidates = candidates[np.isfinite(candidates) & (candidates <= limit)]
    if len(candidates) < 100:
        raise ValueError("not enough PCD support below the flight trajectory to infer a floor")
    lower, upper = np.percentile(candidates, [0.5, 99.5])
    edges = np.arange(np.floor(lower / bin_size) * bin_size,
                      np.ceil(upper / bin_size) * bin_size + 2 * bin_size, bin_size)
    counts, edges = np.histogram(candidates, edges)
    index = int(np.argmax(counts))
    return float(0.5 * (edges[index] + edges[index + 1])), {
        "method": "strongest_horizontal_pcd_support_below_trajectory",
        "candidate_upper_z_m": limit,
        "histogram_bin_m": bin_size,
        "peak_points": int(counts[index]),
    }


def infer_ceiling_z(points: np.ndarray, trajectory: np.ndarray, floor_z: float,
                    bin_size: float = 0.05) -> tuple[float, dict]:
    limit = max(floor_z + 2.0, float(np.median(trajectory[:, 2]) + 1.0))
    candidates = points[:, 2]
    candidates = candidates[np.isfinite(candidates) & (candidates >= limit)]
    if len(candidates) < 100:
        raise ValueError("not enough upper PCD support to infer a ceiling")
    lower, upper = np.percentile(candidates, [0.5, 99.5])
    edges = np.arange(np.floor(lower / bin_size) * bin_size,
                      np.ceil(upper / bin_size) * bin_size + 2 * bin_size, bin_size)
    counts, edges = np.histogram(candidates, edges)
    index = int(np.argmax(counts))
    return float(0.5 * (edges[index] + edges[index + 1])), {
        "method": "strongest_horizontal_pcd_support_above_trajectory",
        "candidate_lower_z_m": limit, "histogram_bin_m": bin_size,
        "peak_points": int(counts[index]),
    }


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd", type=Path)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("output_prefix", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--floor-z", type=float, default=None)
    parser.add_argument("--trajectory-margin", type=float, default=5.0)
    parser.add_argument("--floor-tolerance", type=float, default=0.12)
    parser.add_argument("--ceiling-z", type=float, default=None)
    parser.add_argument("--room-max-z", type=float, default=2.0)
    parser.add_argument("--wall-min-count", default="auto")
    args = parser.parse_args()

    points, pcd_header = load_binary_pcd_xyz(args.pcd)
    points = points[np.all(np.isfinite(points), axis=1)]
    trajectory = load_trajectory(args.trajectory)
    lower_xy = trajectory[:, :2].min(axis=0) - args.trajectory_margin
    upper_xy = trajectory[:, :2].max(axis=0) + args.trajectory_margin
    in_roi = np.all((points[:, :2] >= lower_xy) & (points[:, :2] <= upper_xy), axis=1)
    cropped = points[in_roi]
    if len(cropped) < 1000:
        raise ValueError("trajectory-bounded PCD contains too few points")

    if args.floor_z is None:
        floor_z, floor_estimate = infer_floor_z(cropped, trajectory)
    else:
        floor_z = float(args.floor_z)
        floor_estimate = {"method": "operator_supplied"}
    if args.ceiling_z is None:
        ceiling_z, ceiling_estimate = infer_ceiling_z(cropped, trajectory, floor_z)
    else:
        ceiling_z = float(args.ceiling_z)
        ceiling_estimate = {"method": "operator_supplied"}
    if ceiling_z - floor_z < 2.0:
        raise ValueError(f"inferred floor-to-ceiling height is implausible: {ceiling_z - floor_z:.2f} m")
    origin = np.floor(lower_xy / args.resolution) * args.resolution
    upper = np.ceil(upper_xy / args.resolution) * args.resolution
    width, height = np.ceil((upper - origin) / args.resolution).astype(int)
    if width * height > 4_000_000:
        raise ValueError(f"room grid is unexpectedly large: {width}x{height}")

    pixels = np.floor((cropped[:, :2] - origin) / args.resolution).astype(np.int32)
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < width)
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
    cropped, pixels = cropped[inside], pixels[inside]

    floor_points = np.abs(cropped[:, 2] - floor_z) <= args.floor_tolerance
    floor_hits = np.zeros((height, width), dtype=np.uint16)
    np.add.at(floor_hits, (pixels[floor_points, 1], pixels[floor_points, 0]), 1)
    observed_floor = (floor_hits > 0).astype(np.uint8)
    # Bridge ordinary LiDAR sampling holes but do not synthesize a whole room
    # from camera rays or semantic boxes.
    observed_floor = cv2.morphologyEx(
        observed_floor, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)), iterations=1,
    )
    observed_floor = cv2.dilate(
        observed_floor, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1,
    ).astype(bool)

    density_selected = cropped[:, 2] <= args.room_max_z
    density_hits = np.zeros((height, width), dtype=np.uint32)
    np.add.at(density_hits, (pixels[density_selected, 1], pixels[density_selected, 0]), 1)
    positive_density = density_hits[density_hits > 0]
    if not len(positive_density):
        raise ValueError("no PCD points remain below --room-max-z")
    if args.wall_min_count == "auto":
        wall_min_count = max(2, int(np.ceil(np.percentile(positive_density, 95.0))))
        threshold_source = "positive_cell_p95"
    else:
        try:
            wall_min_count = int(args.wall_min_count)
        except ValueError as error:
            raise ValueError("--wall-min-count must be 'auto' or a positive integer") from error
        if wall_min_count < 1:
            raise ValueError("--wall-min-count must be positive")
        threshold_source = "operator_supplied"
    wall = (density_hits >= wall_min_count).astype(np.uint8)
    # Fill only one-cell sampling holes. No line fitting or long-range gap
    # completion is allowed, so a measured doorway cannot be bridged.
    wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8))
    floor_u8 = observed_floor.astype(np.uint8)
    frontier = cv2.dilate(floor_u8, np.ones((11, 11), np.uint8)) - cv2.erode(
        floor_u8, np.ones((11, 11), np.uint8))
    component_count, labels = cv2.connectedComponents(wall, 8)
    retained = np.zeros_like(wall)
    retained_components = 0
    for label in range(1, component_count):
        component = labels == label
        if np.any(frontier[component]):
            retained[component] = 1
            retained_components += 1
    wall = retained.astype(bool)

    grid = np.full((height, width), -1, dtype=np.int8)
    grid[observed_floor] = 0
    grid[wall] = 100
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output_prefix.with_suffix(".npy"), grid)
    pcd_hash = sha256(args.pcd)
    metadata = {
        "format": "pre_map_vln.pcd_room_grid.v1", "map_role": "pcd_geometry_room_structure",
        "coordinate_frame": "FAST-LIO world, z-up", "source_pcd": str(args.pcd.resolve()),
        "source_pcd_sha256": pcd_hash, "source_trajectory": str(args.trajectory.resolve()),
        "uses_rgbd": False, "uses_semantics": False,
        "resolution_m": args.resolution, "origin_xy_m": origin.tolist(),
        "width": int(width), "height": int(height), "frame_count": None,
        "free_cells": int(np.count_nonzero(grid == 0)),
        "occupied_cells": int(np.count_nonzero(grid == 100)),
        "unknown_cells": int(np.count_nonzero(grid == -1)),
        "floor_z_m": floor_z, "floor_estimate": floor_estimate,
        "ceiling_z_m": ceiling_z, "ceiling_estimate": ceiling_estimate,
        "trajectory_roi_xy_m": {"min": lower_xy.tolist(), "max": upper_xy.tolist(),
                                "margin_m": args.trajectory_margin},
        "wall_extraction": {
            "algorithm": "bounded_xy_density_boundary_components_v4",
            "room_max_world_z_m": args.room_max_z,
            "minimum_points_per_cell": wall_min_count,
            "threshold_source": threshold_source,
            "density_percentile_if_auto": 95.0,
            "density_selected_points": int(np.count_nonzero(density_selected)),
            "line_completion": False,
            "retained_boundary_component_count": retained_components,
            "source_points": pcd_header["points"], "finite_points": int(len(points)),
            "roi_points": int(len(cropped)), "wall_cells": int(np.count_nonzero(wall)),
        },
    }
    atomic_json(args.output_prefix.with_suffix(".json"), metadata)
    preview = np.zeros((height, width, 3), dtype=np.uint8)
    preview[grid == -1] = (90, 90, 90)
    preview[grid == 0] = (245, 245, 245)
    preview[grid == 100] = (20, 20, 20)
    Image.fromarray(np.flipud(preview)).save(args.output_prefix.with_suffix(".png"))
    print(json.dumps(metadata, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
