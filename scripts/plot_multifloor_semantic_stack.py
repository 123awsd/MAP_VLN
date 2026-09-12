#!/usr/bin/env python3
"""Render the room-semantic floors with the same coordinates as the point stack."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors as mpl_colors


SEMANTIC_COLORS = {
    "bedroom": "#4e79a7", "living_room": "#59a14f", "kitchen": "#f28e2b",
    "dining_room": "#edc948", "bathroom": "#76b7b2", "office": "#b07aa1",
    "laundry_utility": "#9c755f", "entry_hall": "#bab0ac",
    "corridor": "#86bcb6", "unknown": "#c7c7c7", "stairwell": "#e15759",
}

WALL_RGB = np.asarray((107, 114, 128), dtype=float) / 255.0
FREE_RGB = np.asarray((224, 224, 224), dtype=float) / 255.0


def add_soft_image_shadow(path: Path, strength: float, blur_px: float,
                          offset_px: tuple[int, int]) -> None:
    """Add a screen-space shadow behind rendered pixels without altering geometry."""
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    darkness = 255.0 - image.min(axis=2).astype(np.float32)
    foreground_alpha = np.clip((darkness - 2.0) / 18.0, 0.0, 1.0)
    mask = (darkness > 5.0).astype(np.uint8) * 255
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
    )
    shadow = cv2.GaussianBlur(mask, (0, 0), blur_px).astype(np.float32) / 255.0
    transform = np.float32([[1, 0, offset_px[0]], [0, 1, offset_px[1]]])
    shadow = cv2.warpAffine(shadow, transform, (image.shape[1], image.shape[0]))
    shadow_alpha = np.clip(shadow * strength, 0.0, 1.0)[..., None]
    background = 255.0 * (1.0 - shadow_alpha) + 72.0 * shadow_alpha
    alpha = foreground_alpha[..., None]
    composed = image.astype(np.float32) * alpha + background * (1.0 - alpha)
    cv2.imwrite(str(path), np.clip(composed, 0, 255).astype(np.uint8))


def semantic_raster(grid: np.ndarray, origin: np.ndarray, resolution: float,
                    rooms: list[dict], color_lighten: float) -> np.ndarray:
    """Rasterize room polygons to observed free space and close wall-edge gaps."""
    rgba = np.zeros((*grid.shape, 4), dtype=float)
    rgba[grid == 0, :3] = FREE_RGB
    rgba[grid == 0, 3] = 1.0
    free_visual_mask = cv2.dilate(
        (grid == 0).astype(np.uint8), np.ones((5, 5), dtype=np.uint8), iterations=1,
    ).astype(bool)
    for room in rooms:
        polygon = np.asarray(room.get("polygon_xy_m", []), dtype=float)
        if len(polygon) < 3:
            continue
        pixel_polygon = np.round((polygon - origin) / resolution).astype(np.int32)
        room_mask = np.zeros(grid.shape, dtype=np.uint8)
        cv2.fillPoly(room_mask, [pixel_polygon], 1)
        room_mask = cv2.dilate(
            room_mask, np.ones((7, 7), dtype=np.uint8), iterations=1,
        ).astype(bool) & free_visual_mask
        label = room.get("semantic_type", room.get("qwen_semantic_type", "unknown"))
        base_color = np.asarray(mpl_colors.to_rgb(
            SEMANTIC_COLORS.get(label, "#c7c7c7")
        ))
        display_color = base_color * (1.0 - color_lighten) + color_lighten
        rgba[room_mask, :3] = display_color
        rgba[room_mask, 3] = 1.0
        inner_edge = room_mask & ~cv2.erode(
            room_mask.astype(np.uint8), np.ones((3, 3), dtype=np.uint8), iterations=1,
        ).astype(bool)
        rgba[inner_edge, :3] = .96
        rgba[inner_edge, 3] = 1.0
    # Walls are authoritative and are drawn last in the raster.
    rgba[grid == 100, :3] = WALL_RGB
    rgba[grid == 100, 3] = 1.0
    return rgba


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("grid_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--floors", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--stack-gap", type=float, default=2.0)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=26.0)
    parser.add_argument("--z-aspect", type=float, default=1.45)
    parser.add_argument("--wall-size", type=float, default=2.0)
    parser.add_argument("--color-lighten", type=float, default=0.0,
                        help="Blend semantic colors toward white; range 0 to 1")
    parser.add_argument("--shadow-strength", type=float, default=.16)
    parser.add_argument("--shadow-blur", type=float, default=18.0,
                        help="Screen-space shadow blur in pixels")
    parser.add_argument("--shadow-offset", type=int, nargs=2, default=[12, 12],
                        metavar=("X", "Y"), help="Screen-space shadow offset in pixels")
    args = parser.parse_args()
    if not 0.0 <= args.color_lighten <= 1.0:
        parser.error("--color-lighten must be between 0 and 1")

    floor_data = []
    all_xy = []
    for index, floor in enumerate(sorted(args.floors)):
        grid_meta = json.loads((args.grid_dir / f"L{floor}_wall_grid.json").read_text())
        grid = np.load(args.grid_dir / f"L{floor}_wall_grid.npy")
        origin = np.asarray(grid_meta["origin_xy_m"], dtype=float)
        resolution = float(grid_meta["resolution_m"])
        height, width = grid.shape
        graph = json.loads((args.scene_dir / f"L{floor}_scene_graph_qwen.json").read_text())
        rgba = semantic_raster(
            grid, origin, resolution, graph["rooms"], args.color_lighten,
        )
        x_edges = origin[0] + np.arange(width + 1) * resolution
        y_edges = origin[1] + np.arange(height + 1) * resolution
        xx, yy = np.meshgrid(x_edges, y_edges)
        floor_data.append((index, floor, xx, yy, rgba))
        all_xy.append(np.asarray([
            [x_edges[0], y_edges[0]], [x_edges[-1], y_edges[-1]],
        ]))

    xy = np.concatenate([a for a in all_xy if len(a)])
    xmin, ymin = xy.min(axis=0); xmax, ymax = xy.max(axis=0)
    padx, pady = (xmax - xmin) * .05, (ymax - ymin) * .05
    figure = plt.figure(figsize=(12, 10), facecolor="white")
    axis = figure.add_subplot(111, projection="3d")
    for index, floor, xx, yy, rgba in floor_data:
        z = index * args.stack_gap
        zz = np.full_like(xx, z, dtype=float)
        axis.plot_surface(
            xx, yy, zz, rstride=1, cstride=1, facecolors=rgba,
            shade=False, antialiased=False, linewidth=0, rasterized=True,
        )

    axis.set_xlim(xmin - padx, xmax + padx); axis.set_ylim(ymin - pady, ymax + pady)
    axis.set_zlim(-args.stack_gap * .2, args.stack_gap * (len(floor_data) - 1) + args.stack_gap * .3)
    axis.set_box_aspect((xmax - xmin, ymax - ymin, args.stack_gap * len(floor_data) * args.z_aspect))
    axis.set_proj_type("ortho")
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.set_axis_off(); figure.subplots_adjust(left=0, right=1, bottom=0, top=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=220, facecolor="white", bbox_inches="tight", pad_inches=.02)
    plt.close(figure)
    if args.shadow_strength > 0:
        add_soft_image_shadow(
            args.output, args.shadow_strength, args.shadow_blur,
            tuple(args.shadow_offset),
        )
    report = {
        "format": "pre_map_vln.multifloor_semantic_stack.v1",
        "scene_dir": str(args.scene_dir.resolve()), "grid_dir": str(args.grid_dir.resolve()),
        "floors": sorted(args.floors), "stack_gap": args.stack_gap,
        "azimuth": args.azimuth, "elevation": args.elevation, "z_aspect": args.z_aspect,
        "fill_mode": "rasterized_dilated_to_free_space_with_wall_overlay",
        "style": {
            "room_gradient": False, "inner_white_outline_px": 1,
            "color_lighten": args.color_lighten,
            "shadow_strength": args.shadow_strength,
            "shadow_blur": args.shadow_blur,
            "shadow_offset_px": args.shadow_offset,
            "shadow_mode": "screen_space_postprocess",
        },
        "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
