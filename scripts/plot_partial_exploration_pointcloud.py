#!/usr/bin/env python3
"""Render a paper figure of partial RGB-D exploration against the final map.

Colored points are reconstructed from RGB-D frames available at a chosen
temporal progress. Gray points are final occupied-map points that had not yet
been observed at that time. The final map is used only as retrospective
visualization context; it is not an input to the explorer.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from scipy.spatial import cKDTree


def quaternion_matrix_xyzw(value: np.ndarray) -> np.ndarray:
    x, y, z, w = map(float, value)
    norm = x * x + y * y + z * z + w * w
    scale = 2.0 / norm
    return np.asarray([
        [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
        [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
        [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
    ], dtype=np.float64)


def read_ascii_pcd_xyz(path: Path) -> np.ndarray:
    lines = path.read_bytes().splitlines()
    data_line = next(index for index, line in enumerate(lines) if line.startswith(b"DATA"))
    if lines[data_line].split()[1].lower() != b"ascii":
        raise ValueError(f"only ASCII PCD is supported: {path}")
    return np.loadtxt(lines[data_line + 1 :], dtype=np.float32, usecols=(0, 1, 2))


def voxel_first(points: np.ndarray, voxel: float, colors: np.ndarray | None = None):
    keys = np.floor(points / voxel).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    indices.sort()
    return points[indices], None if colors is None else colors[indices]


def rgbd_points(frame_path: Path, stride: int, maximum_depth: float):
    with np.load(frame_path) as frame:
        depth = frame["depth_m"][::stride, ::stride].astype(np.float64)
        rgb = frame["rgb"][::stride, ::stride].reshape(-1, 3)
        position = frame["position"].astype(np.float64)
        rotation = quaternion_matrix_xyzw(frame["orientation_xyzw"])
    rows, columns = np.mgrid[0:480:stride, 0:640:stride]
    z = depth.reshape(-1)
    valid = np.isfinite(z) & (z > 0.15) & (z <= maximum_depth)
    optical = np.stack(((columns.reshape(-1) - 320.0) * z / 320.0,
                        (rows.reshape(-1) - 240.0) * z / 320.0, z), axis=1)
    world = optical[valid] @ rotation.T + position
    return world.astype(np.float32), (rgb[valid].astype(np.float32) / 255.0), position


def equal_3d_limits(axis, points: np.ndarray) -> None:
    low = np.percentile(points, 0.2, axis=0)
    high = np.percentile(points, 99.8, axis=0)
    center = 0.5 * (low + high)
    radius = 0.52 * float(np.max(high - low))
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(max(-0.2, low[2] - 0.2), high[2] + 0.25)
    axis.set_box_aspect((1, 1, max(0.3, (high[2] - low[2]) / (2 * radius))))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--progress", type=float, default=0.5)
    parser.add_argument("--frame-stride", type=int, default=6)
    parser.add_argument("--maximum-depth", type=float, default=6.0)
    parser.add_argument("--color-voxel", type=float, default=0.055)
    parser.add_argument("--context-voxel", type=float, default=0.07)
    parser.add_argument("--observed-radius", type=float, default=0.12)
    parser.add_argument("--elevation", type=float, default=24.0)
    parser.add_argument("--azimuth", type=float, default=-64.0)
    args = parser.parse_args()
    if not 0.0 < args.progress < 1.0:
        parser.error("--progress must be strictly between 0 and 1")

    episode = args.run_dir / "episode"
    frames = sorted(episode.glob("frame_*.npz"))
    if not frames:
        raise FileNotFoundError(f"no frame_*.npz in {episode}")
    cutoff_count = max(1, int(np.ceil(len(frames) * args.progress)))
    selected = frames[:cutoff_count]
    cutoff_sequence = int(selected[-1].stem.split("_")[-1])

    points_parts, color_parts, trajectory = [], [], []
    for frame_path in selected:
        points, colors, position = rgbd_points(frame_path, args.frame_stride, args.maximum_depth)
        points_parts.append(points)
        color_parts.append(colors)
        trajectory.append(position)
    colored, colors = voxel_first(
        np.concatenate(points_parts), args.color_voxel, np.concatenate(color_parts)
    )
    trajectory = np.asarray(trajectory)

    final_map = read_ascii_pcd_xyz(args.run_dir / "bag_export" / "map_occupied.pcd")
    final_map, _ = voxel_first(final_map, args.context_voxel)
    distance, _ = cKDTree(colored).query(final_map, k=1, workers=-1)
    unexplored = final_map[distance > args.observed_radius]

    figure = plt.figure(figsize=(10.2, 7.6), dpi=220, facecolor="white")
    axis = figure.add_subplot(111, projection="3d")
    axis.set_facecolor("white")
    axis.scatter(unexplored[:, 0], unexplored[:, 1], unexplored[:, 2], s=0.45,
                 c="#a7abb1", alpha=0.18, linewidths=0, depthshade=False, rasterized=True)
    axis.scatter(colored[:, 0], colored[:, 1], colored[:, 2], s=0.72,
                 c=colors, alpha=0.92, linewidths=0, depthshade=False, rasterized=True)
    axis.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color="#0878c9",
              linewidth=1.25, alpha=0.95, zorder=8)
    axis.scatter(*trajectory[0], s=38, c="#19a55a", edgecolors="white", linewidths=.7,
                 depthshade=False, zorder=10)
    axis.scatter(*trajectory[-1], s=42, c="#e23b3b", marker="D", edgecolors="white",
                 linewidths=.7, depthshade=False, zorder=10)
    equal_3d_limits(axis, final_map)
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.set_xlabel("X (m)", labelpad=7)
    axis.set_ylabel("Y (m)", labelpad=7)
    axis.set_zlabel("Z (m)", labelpad=5)
    axis.grid(False)
    for pane in (axis.xaxis.pane, axis.yaxis.pane, axis.zaxis.pane):
        pane.set_facecolor((1, 1, 1, 0)); pane.set_edgecolor("#d5d8dc")
    axis.set_title(f"Exploration state at {args.progress:.0%} of the run", fontsize=15,
                   fontweight="semibold", pad=14)
    axis.text2D(.02, .965,
                f"Scene 00337  |  cutoff frame {cutoff_sequence}  |  colored: observed by cutoff",
                transform=axis.transAxes, fontsize=9, color="#48515b")
    legend = [
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor="#3c94c6",
               markeredgecolor="none", markersize=7, label="Observed RGB-D surface"),
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor="#a7abb1",
               markeredgecolor="none", markersize=7, label="Not yet observed"),
        Line2D([0], [0], color="#0878c9", linewidth=1.6, label="Executed trajectory"),
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor="#19a55a",
               markeredgecolor="white", markersize=7, label="Start"),
        Line2D([0], [0], marker="D", linestyle="", markerfacecolor="#e23b3b",
               markeredgecolor="white", markersize=6, label="50% state"),
    ]
    axis.legend(handles=legend, loc="lower left", bbox_to_anchor=(.015, .015), frameon=True,
                framealpha=.94, facecolor="white", edgecolor="#d7dadd", fontsize=8.5)
    figure.subplots_adjust(left=.01, right=.98, bottom=.03, top=.92)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=300, bbox_inches="tight", pad_inches=.08, facecolor="white")
    pdf = args.output.with_suffix(".pdf")
    figure.savefig(pdf, bbox_inches="tight", pad_inches=.08, facecolor="white")
    plt.close(figure)
    metadata = {
        "format": "pre_map_vln.partial_exploration_figure.v1",
        "source_run": str(args.run_dir.resolve()),
        "progress": args.progress,
        "selected_recorded_frames": len(selected),
        "total_recorded_frames": len(frames),
        "cutoff_sequence": cutoff_sequence,
        "colored_point_count": int(len(colored)),
        "not_yet_observed_point_count": int(len(unexplored)),
        "definition": "Gray points are final occupied-map points farther than observed_radius from RGB-D points accumulated by the temporal cutoff.",
        "final_map_is_retrospective_context_only": True,
        "png": str(args.output.resolve()), "pdf": str(pdf.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
