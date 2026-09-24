#!/usr/bin/env python3
"""Build a sparse 3-D occupied-voxel map from the cleaned roofless PLY."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
from scipy.spatial import cKDTree

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from build_reconstruction_occupancy_grid import read_binary_ply


def write_voxel_ply(
    path: Path,
    centers: np.ndarray,
    rgb: np.ndarray,
    evidence: np.ndarray,
) -> None:
    vertices = np.empty(
        len(centers),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
            ("evidence", "<u2"),
        ],
    )
    vertices["x"], vertices["y"], vertices["z"] = centers.T
    vertices["red"], vertices["green"], vertices["blue"] = rgb.T
    vertices["evidence"] = np.minimum(evidence, 65535).astype(np.uint16)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment sparse occupied voxel centers\n"
        f"element vertex {len(vertices)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "property ushort evidence\nend_header\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode())
        vertices.tofile(stream)


def vivid_height_colors(
    relative_height: np.ndarray,
    min_height: float = 0.0,
    max_height: float = 3.5,
) -> np.ndarray:
    """Map local height to the shared red-to-purple vivid gradient."""
    normalized = np.clip(
        (relative_height - min_height) / (max_height - min_height), 0.0, 1.0
    )
    return (plt.get_cmap("turbo_r")(normalized)[:, :3] * 255.0).astype(np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_ply", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--min-points", type=int, default=2)
    parser.add_argument(
        "--occupancy-2d",
        type=Path,
        help="2-D occupancy NPZ used to fill the floor (defaults beside output_dir)",
    )
    args = parser.parse_args()
    if args.resolution <= 0 or args.min_points < 1:
        parser.error("resolution and min-points must be positive")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    points = read_binary_ply(args.input_ply)
    xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float32)
    origin = np.floor(xyz.min(axis=0) / args.resolution) * args.resolution
    indices = np.floor((xyz - origin) / args.resolution).astype(np.int32)
    unique_indices, inverse, evidence = np.unique(
        indices, axis=0, return_inverse=True, return_counts=True
    )
    keep = evidence >= args.min_points
    unique_indices = unique_indices[keep]
    evidence = evidence[keep]
    occupied_before_floor_fill = len(unique_indices)

    occupancy_2d_path = args.occupancy_2d
    if occupancy_2d_path is None:
        occupancy_2d_path = args.output_dir.parent / "occupancy_2d" / "occupancy_grid.npz"
    if not occupancy_2d_path.is_file():
        raise FileNotFoundError(occupancy_2d_path)
    occupancy_2d = np.load(occupancy_2d_path, allow_pickle=False)
    grid_2d = occupancy_2d["occupancy"]
    origin_2d = occupancy_2d["origin"]
    resolution_2d = float(occupancy_2d["resolution"])
    free_y, free_x = np.nonzero(grid_2d == 0)
    floor_xy = np.column_stack(
        (
            origin_2d[0] + (free_x + 0.5) * resolution_2d,
            origin_2d[1] + (free_y + 0.5) * resolution_2d,
        )
    )
    ground = points[points["label"] == 1]
    ground_xyz = np.column_stack((ground["x"], ground["y"], ground["z"]))
    ground_tree = cKDTree(ground_xyz[:, :2])
    _, nearest_ground = ground_tree.query(floor_xy, k=min(5, len(ground_xyz)), workers=-1)
    if nearest_ground.ndim == 1:
        nearest_ground = nearest_ground[:, None]
    floor_z = np.median(ground_xyz[nearest_ground, 2], axis=1)
    floor_xyz = np.column_stack((floor_xy, floor_z))
    floor_indices = np.floor((floor_xyz - origin) / args.resolution).astype(np.int32)

    combined_indices = np.vstack((unique_indices, floor_indices))
    combined_evidence = np.concatenate(
        (evidence, np.full(len(floor_indices), args.min_points, dtype=evidence.dtype))
    )
    unique_indices, merge_inverse = np.unique(
        combined_indices, axis=0, return_inverse=True
    )
    merged_evidence = np.zeros(len(unique_indices), dtype=combined_evidence.dtype)
    np.maximum.at(merged_evidence, merge_inverse, combined_evidence)
    evidence = merged_evidence
    floor_voxels_added = len(unique_indices) - occupied_before_floor_fill
    centers = origin + (unique_indices.astype(np.float32) + 0.5) * args.resolution
    grid_size = unique_indices.max(axis=0) + 1

    # Use one vivid continuous height gradient for the floor, walls, and other
    # structures so their colors join without a boundary at floor level.
    _, nearest_floor_for_voxel = ground_tree.query(
        centers[:, :2], k=min(5, len(ground_xyz)), workers=-1
    )
    if nearest_floor_for_voxel.ndim == 1:
        nearest_floor_for_voxel = nearest_floor_for_voxel[:, None]
    local_floor_z = np.median(ground_xyz[nearest_floor_for_voxel, 2], axis=1)
    relative_height = centers[:, 2] - local_floor_z
    color_min_height = 0.0
    color_max_height = 3.5
    voxel_rgb = vivid_height_colors(
        relative_height, color_min_height, color_max_height
    )

    # Apply exactly the same local-floor-relative gradient to every point in
    # the cleaned roofless cloud, without changing its geometry.
    _, nearest_floor_for_point = ground_tree.query(
        xyz[:, :2], k=min(5, len(ground_xyz)), workers=-1
    )
    if nearest_floor_for_point.ndim == 1:
        nearest_floor_for_point = nearest_floor_for_point[:, None]
    point_floor_z = np.median(ground_xyz[nearest_floor_for_point, 2], axis=1)
    point_relative_height = xyz[:, 2] - point_floor_z
    point_rgb = vivid_height_colors(
        point_relative_height, color_min_height, color_max_height
    )
    gradient_cloud_path = args.output_dir.parent / "map_roofless_height_gradient_5cm.ply"
    write_voxel_ply(
        gradient_cloud_path,
        xyz,
        point_rgb,
        np.ones(len(xyz), dtype=np.uint16),
    )

    ply_path = args.output_dir / "occupied_voxels_10cm.ply"
    npz_path = args.output_dir / "occupied_voxels_10cm.npz"
    preview_path = args.output_dir / "occupied_voxels_10cm_preview.png"
    write_voxel_ply(ply_path, centers, voxel_rgb, evidence)
    np.savez_compressed(
        npz_path,
        occupied_indices=unique_indices,
        colors=voxel_rgb,
        evidence=evidence.astype(np.uint16),
        origin=origin.astype(np.float32),
        resolution=np.float32(args.resolution),
        grid_size=grid_size.astype(np.int32),
    )

    stride = max(1, len(centers) // 150_000)
    shown, colors = centers[::stride], voxel_rgb[::stride] / 255.0
    figure = plt.figure(figsize=(13, 6), dpi=170)
    top = figure.add_subplot(121)
    top.scatter(shown[:, 0], shown[:, 1], c=colors, s=1.0, marker="s")
    top.set_aspect("equal")
    top.set_title("3-D occupied voxels: top-down")
    top.set_xlabel("X [m]")
    top.set_ylabel("Y [m]")
    view = figure.add_subplot(122, projection="3d")
    view.scatter(shown[:, 0], shown[:, 1], shown[:, 2], c=colors, s=0.8, marker="s")
    view.view_init(30, -60)
    view.set_title("10 cm occupied voxels")
    view.set_xlabel("X")
    view.set_ylabel("Y")
    view.set_zlabel("Z")
    figure.tight_layout()
    figure.savefig(preview_path, bbox_inches="tight")
    plt.close(figure)

    report = {
        "source_ply": str(args.input_ply),
        "source_points": len(points),
        "representation": "sparse_occupied_voxels",
        "free_space_modeled": False,
        "resolution": args.resolution,
        "min_source_points_per_voxel": args.min_points,
        "occupied_voxels": len(unique_indices),
        "occupied_voxels_before_floor_fill": occupied_before_floor_fill,
        "floor_free_cells_used": len(floor_indices),
        "floor_voxels_added": floor_voxels_added,
        "floor_height_method": "median_of_5_nearest_clean_ground_points",
        "color_mode": "vivid_continuous_turbo_reversed_floor_to_walls",
        "color_height_range_m": [color_min_height, color_max_height],
        "height_gradient_point_cloud": str(gradient_cloud_path),
        "occupancy_2d_source": str(occupancy_2d_path),
        "origin": origin.tolist(),
        "grid_size": grid_size.tolist(),
        "metric_size": (grid_size * args.resolution).tolist(),
        "outputs": {
            "ply": str(ply_path),
            "npz": str(npz_path),
            "preview": str(preview_path),
        },
    }
    (args.output_dir / "occupancy_3d_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
