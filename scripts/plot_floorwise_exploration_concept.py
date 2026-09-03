#!/usr/bin/env python3
"""Render a clean floor-wise exploration concept using only FALCON points."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from plot_exploded_exploration_progress import exploded, floor_indices
from plot_partial_exploration_pointcloud import read_ascii_pcd_xyz


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("floors_json", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--floor-spacing", type=float, default=3.7)
    parser.add_argument("--top-clearance", type=float, default=2.30)
    parser.add_argument("--azimuth", type=float, default=-63.0)
    parser.add_argument("--elevation", type=float, default=25.0)
    args = parser.parse_args()

    floor_data = json.loads(args.floors_json.read_text())
    floor_z = np.asarray([item["floor_z_m"] for item in floor_data], dtype=np.float32)
    points = read_ascii_pcd_xyz(args.run_dir / "bag_export/map_occupied.pcd")
    indices = floor_indices(points, floor_z, args.top_clearance)
    keep = indices >= 0
    points, indices = points[keep], indices[keep]

    # L1 is fully explored; L3 is unobserved. Split L2 at its median Y so that
    # exactly half of its retained FALCON points form a coherent spatial half.
    second_floor = indices == 1
    split_y = float(np.median(points[second_floor, 1]))
    explored = (indices == 0) | (second_floor & (points[:, 1] <= split_y))
    shown = exploded(points, indices, floor_z, args.floor_spacing)

    with (args.run_dir / "bag_export/trajectory.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    trajectory = np.asarray([[float(row[k]) for k in ("x", "y", "z")] for row in rows])
    trajectory_floor = np.argmin(
        np.abs(trajectory[:, None, 2] - (floor_z[None, :] + 1.2)), axis=1
    )
    # End at the first point reaching the explored spatial half of L2.
    candidates = np.flatnonzero((trajectory_floor == 1) & (trajectory[:, 1] <= split_y))
    cutoff = int(candidates[0]) if len(candidates) else len(trajectory) - 1
    trajectory = trajectory[: cutoff + 1]
    trajectory_floor = trajectory_floor[: cutoff + 1]
    shown_trajectory = exploded(trajectory, trajectory_floor, floor_z, args.floor_spacing)

    fig = plt.figure(figsize=(8.6, 7.2), dpi=220, facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(*shown[~explored].T, s=1.2, marker="s", c="#aeb4ba", alpha=.55,
               linewidths=0, depthshade=False, rasterized=True)
    ax.scatter(*shown[explored].T, s=1.42, marker="s", c="#079bbd", alpha=.98,
               linewidths=0, depthshade=False, rasterized=True)

    for floor in range(len(floor_z)):
        member = trajectory_floor == floor
        starts = np.flatnonzero(member & np.r_[True, ~member[:-1]])
        ends = np.flatnonzero(member & np.r_[~member[1:], True]) + 1
        for start, end in zip(starts, ends):
            if end - start >= 2:
                ax.plot(*shown_trajectory[start:end].T, color="#ee7b2d", linewidth=.82,
                        alpha=.82, zorder=9)
    ax.scatter(*shown_trajectory[-1], s=34, c="#d9363e", marker="D",
               edgecolors="white", linewidths=.65, depthshade=False, zorder=11)

    low = np.percentile(shown, .15, axis=0)
    high = np.percentile(shown, 99.85, axis=0)
    ax.set_xlim(low[0] - .25, high[0] + .25)
    ax.set_ylim(low[1] - .25, high[1] + .25)
    ax.set_zlim(-.15, args.floor_spacing * (len(floor_z) - 1) + args.top_clearance + .15)
    ax.set_box_aspect((high[0] - low[0], high[1] - low[1], 10.0))
    ax.view_init(elev=args.elevation, azim=args.azimuth)
    ax.set_axis_off()
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=300, bbox_inches="tight", pad_inches=0, facecolor="white")
    pdf = args.output.with_suffix(".pdf")
    fig.savefig(pdf, bbox_inches="tight", pad_inches=0, facecolor="white")
    plt.close(fig)
    report = {
        "format": "pre_map_vln.floorwise_exploration_concept.v1",
        "source_point_cloud": str((args.run_dir / "bag_export/map_occupied.pcd").resolve()),
        "source_trajectory": str((args.run_dir / "bag_export/trajectory.csv").resolve()),
        "definition": "Illustrative floor-wise state: L1 complete, spatial half of L2 complete, L3 not started.",
        "not_a_temporal_replay": True,
        "l2_split_axis": "y", "l2_split_y_m": split_y,
        "l2_explored_point_fraction": float(np.mean(explored[second_floor])),
        "shown_point_count": int(len(points)), "trajectory_rows_shown": int(len(trajectory)),
        "png": str(args.output.resolve()), "pdf": str(pdf.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
