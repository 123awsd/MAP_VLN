#!/usr/bin/env python3
"""Paper-style exploded-floor visualization of intermediate exploration."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from scipy.spatial import cKDTree

from plot_partial_exploration_pointcloud import (
    quaternion_matrix_xyzw,
    read_ascii_pcd_xyz,
    voxel_first,
)


def depth_endpoints(path: Path, pixel_stride: int, maximum_depth: float) -> tuple[np.ndarray, np.ndarray]:
    with np.load(path) as frame:
        depth = frame["depth_m"][::pixel_stride, ::pixel_stride].astype(np.float64)
        position = frame["position"].astype(np.float64)
        rotation = quaternion_matrix_xyzw(frame["orientation_xyzw"])
    rows, columns = np.mgrid[0:480:pixel_stride, 0:640:pixel_stride]
    z = depth.reshape(-1)
    valid = np.isfinite(z) & (z > 0.15) & (z <= maximum_depth)
    optical = np.stack(((columns.reshape(-1) - 320.0) * z / 320.0,
                        (rows.reshape(-1) - 240.0) * z / 320.0, z), axis=1)
    return (optical[valid] @ rotation.T + position).astype(np.float32), position


def floor_indices(points: np.ndarray, floor_z: np.ndarray, top_clearance: float) -> np.ndarray:
    difference = points[:, None, 2] - floor_z[None, :]
    valid = (difference >= -0.12) & (difference <= top_clearance)
    cost = np.where(valid, np.abs(difference - 1.0), np.inf)
    indices = np.argmin(cost, axis=1)
    indices[~np.any(valid, axis=1)] = -1
    return indices


def exploded(points: np.ndarray, indices: np.ndarray, floor_z: np.ndarray, spacing: float) -> np.ndarray:
    result = points.copy()
    result[:, 2] = points[:, 2] - floor_z[indices] + spacing * indices
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("floors_json", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--coverage", type=float, default=0.50,
                        help="target fraction of surface points observable across archived frames")
    parser.add_argument("--pixel-stride", type=int, default=9)
    parser.add_argument("--maximum-depth", type=float, default=6.0)
    parser.add_argument("--map-voxel", type=float, default=0.075)
    parser.add_argument("--match-radius", type=float, default=0.13)
    parser.add_argument("--floor-spacing", type=float, default=3.7)
    parser.add_argument("--top-clearance", type=float, default=2.30,
                        help="ceiling cutaway height above each floor")
    args = parser.parse_args()
    if not 0 < args.coverage < 1:
        parser.error("--coverage must be between zero and one")

    frames = sorted((args.run_dir / "episode").glob("frame_*.npz"))
    if not frames:
        raise FileNotFoundError(args.run_dir / "episode")
    final_map = read_ascii_pcd_xyz(args.run_dir / "bag_export/map_occupied.pcd")
    final_map, _ = voxel_first(final_map, args.map_voxel)
    tree = cKDTree(final_map)

    per_frame_indices: list[np.ndarray] = []
    trajectory = []
    observable = np.zeros(len(final_map), dtype=bool)
    for path in frames:
        endpoints, position = depth_endpoints(path, args.pixel_stride, args.maximum_depth)
        distance, index = tree.query(endpoints, k=1, workers=-1)
        matched = np.unique(index[distance <= args.match_radius])
        per_frame_indices.append(matched)
        observable[matched] = True
        trajectory.append(position)
    observable_count = int(observable.sum())
    target = int(np.ceil(args.coverage * observable_count))
    observed = np.zeros(len(final_map), dtype=bool)
    cutoff = 0
    for cutoff, matched in enumerate(per_frame_indices):
        observed[matched] = True
        if int(observed.sum()) >= target:
            break
    cutoff_sequence = int(frames[cutoff].stem.split("_")[-1])
    achieved = float(np.count_nonzero(observed & observable) / observable_count)

    floor_data = json.loads(args.floors_json.read_text())
    floor_z = np.asarray([item["floor_z_m"] for item in floor_data], dtype=np.float32)
    map_floor = floor_indices(final_map, floor_z, args.top_clearance)
    keep = map_floor >= 0
    shown_map = final_map[keep]
    shown_floor = map_floor[keep]
    shown_observed = observed[keep]
    shown_map = exploded(shown_map, shown_floor, floor_z, args.floor_spacing)

    trajectory = np.asarray(trajectory[:cutoff + 1])
    trajectory_floor = np.argmin(np.abs(trajectory[:, None, 2] - (floor_z[None, :] + 1.2)), axis=1)
    shown_trajectory = exploded(trajectory, trajectory_floor, floor_z, args.floor_spacing)

    fig = plt.figure(figsize=(9.2, 7.2), dpi=220, facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    gray, blue, orange = "#b8bdc3", "#0796b8", "#ed7d31"
    ax.scatter(*shown_map[~shown_observed].T, s=.52, c=gray, alpha=.24,
               linewidths=0, depthshade=False, rasterized=True)
    ax.scatter(*shown_map[shown_observed].T, s=.76, c=blue, alpha=.90,
               linewidths=0, depthshade=False, rasterized=True)

    # Plot within-floor pieces only, avoiding artificial vertical lines at transitions.
    for floor in range(len(floor_z)):
        member = trajectory_floor == floor
        starts = np.flatnonzero(member & np.r_[True, ~member[:-1]])
        ends = np.flatnonzero(member & np.r_[~member[1:], True]) + 1
        for start, end in zip(starts, ends):
            if end - start >= 2:
                segment = shown_trajectory[start:end]
                ax.plot(*segment.T, color=orange, linewidth=1.35, alpha=.96, zorder=9)
    ax.scatter(*shown_trajectory[-1], s=48, c="#d83a3a", marker="D",
               edgecolors="white", linewidths=.8, depthshade=False, zorder=11)

    low = np.percentile(shown_map, .2, axis=0)
    high = np.percentile(shown_map, 99.8, axis=0)
    ax.set_xlim(low[0] - .4, high[0] + .4)
    ax.set_ylim(low[1] - .4, high[1] + .4)
    ax.set_zlim(-.2, args.floor_spacing * (len(floor_z) - 1) + args.top_clearance + .2)
    ax.set_box_aspect((high[0] - low[0], high[1] - low[1], 10.0))
    ax.view_init(elev=25, azim=-63)
    ax.set_axis_off()

    for index in range(len(floor_z)):
        local = shown_map[shown_floor == index]
        ax.text(float(local[:, 0].min() - .7), float(local[:, 1].max()),
                args.floor_spacing * index + 1.0, f"L{index + 1}", color="#343b43",
                fontsize=12, fontweight="bold", zorder=15)

    legend = [
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor=blue,
               markeredgecolor="none", markersize=7, label="Explored"),
        Line2D([0], [0], marker="o", linestyle="", markerfacecolor=gray,
               markeredgecolor="none", markersize=7, label="Not yet explored"),
        Line2D([0], [0], color=orange, linewidth=2, label="Executed trajectory"),
        Line2D([0], [0], marker="D", linestyle="", markerfacecolor="#d83a3a",
               markeredgecolor="white", markersize=7, label="Current position"),
    ]
    ax.legend(handles=legend, loc="upper center", bbox_to_anchor=(.50, 1.015), ncol=4,
              frameon=False, fontsize=9.5, handletextpad=.45, columnspacing=1.3)
    ax.text2D(.5, .025,
              f"00337  |  {achieved:.0%} surface coverage  |  frame {cutoff_sequence}",
              transform=ax.transAxes, ha="center", color="#515961", fontsize=9)
    fig.subplots_adjust(left=.01, right=.99, bottom=.01, top=.96)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=300, bbox_inches="tight", pad_inches=.03, facecolor="white")
    pdf = args.output.with_suffix(".pdf")
    fig.savefig(pdf, bbox_inches="tight", pad_inches=.03, facecolor="white")
    plt.close(fig)
    report = {
        "format": "pre_map_vln.exploded_exploration_progress.v1",
        "target_coverage": args.coverage,
        "achieved_coverage": achieved,
        "coverage_denominator": "final-map surface points observable from all archived RGB-D frames",
        "cutoff_recorded_frame_index": cutoff,
        "cutoff_sequence": cutoff_sequence,
        "total_recorded_frames": len(frames),
        "observable_final_map_points": observable_count,
        "observed_at_cutoff": int(np.count_nonzero(observed & observable)),
        "ceiling_cutaway_height_m": args.top_clearance,
        "floor_display_spacing_m": args.floor_spacing,
        "final_map_is_retrospective_context_only": True,
        "png": str(args.output.resolve()), "pdf": str(pdf.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
