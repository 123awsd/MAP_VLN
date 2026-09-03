#!/usr/bin/env python3
"""Render a floor point cloud with the exact extent of a room-grid figure."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

from plot_partial_exploration_pointcloud import read_ascii_pcd_xyz


def render(points, extent, floor_z, output, clean):
    figure, axis = plt.subplots(1, 1, figsize=(9, 9), facecolor="white")
    relative_z = points[:, 2] - floor_z
    # Project each real 0.1 m voxel center into one 0.1 m image cell. Taking
    # the highest return per XY cell gives a dense top view without spatial
    # interpolation or synthetic points.
    voxel = 0.10
    width = int(np.ceil((extent[1] - extent[0]) / voxel))
    height = int(np.ceil((extent[3] - extent[2]) / voxel))
    columns = np.floor((points[:, 0] - extent[0]) / voxel).astype(int)
    rows = np.floor((points[:, 1] - extent[2]) / voxel).astype(int)
    valid = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    projection = np.full((height, width), -np.inf, dtype=np.float32)
    np.maximum.at(projection, (rows[valid], columns[valid]), relative_z[valid])
    projection[~np.isfinite(projection)] = np.nan
    dark_height = LinearSegmentedColormap.from_list(
        "dark_height", ["#111827", "#123f5a", "#086a78", "#058493"]
    )
    dark_height.set_bad("white")
    axis.imshow(
        projection, origin="lower", extent=extent, interpolation="nearest",
        cmap=dark_height, vmin=0.15, vmax=2.30, rasterized=True,
    )
    axis.set_xlim(extent[0], extent[1])
    axis.set_ylim(extent[2], extent[3])
    axis.set_aspect("equal")
    axis.grid(False)
    if clean:
        axis.axis("off")
        figure.subplots_adjust(left=0, right=1, bottom=0, top=1)
    else:
        axis.set_title("L2 | occupied point cloud (top view)")
        axis.set_xlabel("FALCON X (m)")
        axis.set_ylabel("FALCON Y (m)")
        figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight", pad_inches=0 if clean else 0.1)
    figure.savefig(output.with_suffix(".pdf"), bbox_inches="tight", pad_inches=0 if clean else 0.1)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd", type=Path)
    parser.add_argument("grid_metadata", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--lower", type=float, default=0.15)
    parser.add_argument("--upper", type=float, default=2.65)
    args = parser.parse_args()
    metadata = json.loads(args.grid_metadata.read_text())
    floor_z = float(metadata["floor_z_m"])
    origin = np.asarray(metadata["origin_xy_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    extent = (
        origin[0], origin[0] + int(metadata["width"]) * resolution,
        origin[1], origin[1] + int(metadata["height"]) * resolution,
    )
    cloud = read_ascii_pcd_xyz(args.pcd)
    keep = (
        (cloud[:, 2] >= floor_z + args.lower)
        & (cloud[:, 2] <= floor_z + args.upper)
        & (cloud[:, 0] >= extent[0]) & (cloud[:, 0] <= extent[1])
        & (cloud[:, 1] >= extent[2]) & (cloud[:, 1] <= extent[3])
    )
    selected = cloud[keep]
    if not len(selected):
        raise RuntimeError("No points remain in requested floor band")
    render(selected, extent, floor_z, args.output, clean=False)
    clean_output = args.output.with_name(args.output.stem + "_clean" + args.output.suffix)
    render(selected, extent, floor_z, clean_output, clean=True)
    report = {
        "format": "pre_map_vln.floor_pointcloud_topdown.v1",
        "floor": int(metadata.get("floor", 2)), "floor_z_m": floor_z,
        "z_band_m": [floor_z + args.lower, floor_z + args.upper],
        "xy_extent_m": list(extent), "point_count": int(len(selected)),
        "source_pcd": str(args.pcd.resolve()), "grid_metadata": str(args.grid_metadata.resolve()),
        "labeled_png": str(args.output.resolve()), "clean_png": str(clean_output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
