#!/usr/bin/env python3
"""Export original-data, per-floor top-down figures for Stage 1."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as MplPolygon

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))
from plot_dense_rgbd_floor_pointcloud import depth_points  # noqa: E402
from plot_partial_exploration_pointcloud import voxel_first  # noqa: E402
from plot_multifloor_box_stack import box_corners, category_group, PASTEL_GROUPS  # noqa: E402


def extent(meta: dict, grid: np.ndarray) -> tuple[float, float, float, float]:
    origin = np.asarray(meta["origin_xy_m"], dtype=float)
    res = float(meta["resolution_m"])
    return origin[0], origin[0] + grid.shape[1] * res, origin[1], origin[1] + grid.shape[0] * res


def save_base(ax, grid: np.ndarray, meta: dict) -> None:
    x0, x1, y0, y1 = extent(meta, grid)
    wall = np.ma.masked_where(grid != 100, grid)
    ax.imshow(wall, origin="lower", extent=(x0, x1, y0, y1), cmap="Greys", vmin=0, vmax=100,
              interpolation="nearest", alpha=.72)
    ax.set_aspect("equal")
    ax.set_axis_off()


def render_cloud(ax, episode: Path, floor_z: float, box: tuple[float, float, float, float], voxel: float) -> int:
    parts = []
    for frame in sorted(episode.glob("frame_*.npz")):
        points = depth_points(frame, 2, 5.0)
        keep = ((points[:, 2] >= floor_z + .10) & (points[:, 2] <= floor_z + 2.30) &
                (points[:, 0] >= box[0]) & (points[:, 0] <= box[1]) &
                (points[:, 1] >= box[2]) & (points[:, 1] <= box[3]))
        if np.any(keep):
            parts.append(points[keep])
    raw = np.concatenate(parts)
    fused, _ = voxel_first(raw, voxel)
    ax.scatter(fused[:, 0], fused[:, 1], s=.20, c="#164e70", alpha=.78, linewidths=0, rasterized=True)
    return len(fused)


def render_rooms(ax, graph: dict) -> int:
    colors = {
        "bedroom": "#82a9ce", "living_room": "#8fbe82", "kitchen": "#f5b36b",
        "dining_room": "#edd889", "bathroom": "#8fc9c5", "office": "#c29bc9",
        "entry_hall": "#c8c0bb", "corridor": "#9fc7c2", "unknown": "#d0d0d0", "stairwell": "#e98b8b",
    }
    count = 0
    for room in graph.get("rooms", []):
        polygon = np.asarray(room.get("polygon_xy_m", []), dtype=float)
        if len(polygon) < 3:
            continue
        color = colors.get(room.get("semantic_type", "unknown"), "#d0d0d0")
        ax.add_patch(MplPolygon(polygon, closed=True, facecolor=color, edgecolor="white", linewidth=.6, alpha=.92))
        count += 1
    return count


def render_boxes(ax, rows: list[dict[str, str]]) -> int:
    for row in rows:
        polygon = box_corners(row)
        group = category_group(row["name"])
        color = PASTEL_GROUPS[group]
        ax.add_patch(MplPolygon(polygon, closed=True, facecolor=color, edgecolor="#6b7280", linewidth=.7, alpha=.72))
    return len(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("grid_dir", type=Path)
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("box_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--floors", type=int, nargs="+", default=[1, 2, 3])
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {"format": "pre_map_vln.stage1_floorwise_topdown.v1", "floors": {}}
    for floor in sorted(args.floors):
        meta = json.loads((args.grid_dir / f"L{floor}_wall_grid.json").read_text())
        grid = np.load(args.grid_dir / f"L{floor}_wall_grid.npy")
        x0, x1, y0, y1 = extent(meta, grid)
        floor_z = float(meta["floor_z_m"])
        graph = json.loads((args.scene_dir / f"L{floor}_scene_graph_qwen.json").read_text())
        with (args.box_dir / f"L{floor}_boxes.csv").open(newline="") as stream:
            boxes = list(csv.DictReader(stream))
        stages = {
            "01_点云建图": (lambda ax: render_cloud(ax, args.episode, floor_z, (x0, x1, y0, y1), .025), "点云建图"),
            "02_房间分割": (lambda ax: render_rooms(ax, graph), "房间分割"),
            "03_目标框检测": (lambda ax: render_boxes(ax, boxes), "目标框检测"),
        }
        floor_report = {}
        for dirname, (renderer, title) in stages.items():
            figure, axis = plt.subplots(figsize=(9, 8), facecolor="white")
            save_base(axis, grid, meta)
            count = renderer(axis)
            axis.set_xlim(x0, x1); axis.set_ylim(y0, y1)
            english_title = {
                "点云建图": "Point-cloud mapping",
                "房间分割": "Room segmentation",
                "目标框检测": "Object boxes",
            }[title]
            axis.set_title(f"Scene 00337 | L{floor} | {english_title}", fontsize=15, pad=12)
            path = args.output_dir / dirname / f"场景00337_L{floor}_{title}_俯视原始数据版.png"
            path.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(path, dpi=220, bbox_inches="tight", pad_inches=.08, facecolor="white")
            plt.close(figure)
            floor_report[title] = {"png": str(path.resolve()), "count": int(count)}
        report["floors"][str(floor)] = floor_report
    (args.output_dir / "场景00337_第一阶段_分楼层俯视图.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
