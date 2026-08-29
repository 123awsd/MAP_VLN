#!/usr/bin/env python3
"""Extract per-floor structural walls from a FALCON occupied voxel cloud.

This is geometry-only: no Habitat truth, semantic labels, or object boxes are
used.  A wall column must have occupied support near the floor, extend to the
upper wall band, and remain vertically continuous.  Small high-continuity
components are retained as possible pillars; low furniture is rejected.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_ascii_pcd(path: Path) -> np.ndarray:
    lines = path.read_bytes().splitlines()
    data = next(i for i, line in enumerate(lines) if line.startswith(b"DATA"))
    if lines[data].split()[1].lower() != b"ascii":
        raise ValueError("wall extraction currently expects an ASCII XYZ PCD")
    return np.loadtxt(lines[data + 1:], dtype=np.float32, usecols=(0, 1, 2))


def longest_consecutive_run(values: np.ndarray) -> int:
    if not len(values):
        return 0
    values = np.unique(values)
    breaks = np.flatnonzero(np.diff(values) > 1)
    bounds = np.concatenate(([-1], breaks, [len(values) - 1]))
    return int(np.max(np.diff(bounds)))


def grouped_columns(points: np.ndarray, floor_z: float, voxel: float, upper: float):
    selected = points[
        (points[:, 2] >= floor_z + 0.15)
        & (points[:, 2] <= floor_z + upper)
    ]
    xy = np.rint(selected[:, :2] / voxel).astype(np.int32)
    zb = np.rint((selected[:, 2] - floor_z) / voxel).astype(np.int16)
    order = np.lexsort((zb, xy[:, 1], xy[:, 0]))
    xy, zb = xy[order], zb[order]
    unique_xy, starts = np.unique(xy, axis=0, return_index=True)
    ends = np.concatenate((starts[1:], [len(xy)]))
    result = {}
    for cell, start, end in zip(unique_xy, starts, ends):
        heights = np.unique(zb[start:end])
        result[(int(cell[0]), int(cell[1]))] = heights
    return result


def estimate_global_ceiling(points, floor_z, voxel):
    relative = points[:, 2] - floor_z
    upper = relative[(relative >= 2.15) & (relative <= 3.15)]
    if not len(upper):
        return 2.70
    bins = np.arange(2.10, 3.21, voxel)
    counts, edges = np.histogram(upper, bins)
    centers = (edges[:-1] + edges[1:]) * .5
    # Prefer the lowest member of a near-tied slab peak. This avoids selecting
    # furniture or structure from the next floor above the actual ceiling.
    eligible = np.flatnonzero(counts >= .85 * counts.max())
    return float(centers[eligible[0]])


def local_ceiling_for(cell, occupied_columns, global_ceiling, voxel):
    samples = []
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            heights = occupied_columns.get((cell[0] + dx, cell[1] + dy))
            if heights is None:
                continue
            relative = heights.astype(np.float32) * voxel
            upper = relative[
                (relative >= global_ceiling - .45)
                & (relative <= global_ceiling + .45)
            ]
            if len(upper):
                samples.extend(upper.tolist())
    return global_ceiling if not samples else float(np.median(samples))


def low_connected_top(heights, voxel, maximum_gap=.31):
    relative = heights.astype(np.float32) * voxel
    starts = np.flatnonzero(relative <= .55)
    if not len(starts):
        return None
    top = float(relative[starts[-1]])
    for value in relative[starts[-1] + 1:]:
        if float(value) - top > maximum_gap:
            break
        top = float(value)
    return top


def extract_wall_mask(points, free_points, floor_z, metadata, voxel=.10, minimum_span=1.00):
    import cv2
    height, width = int(metadata["height"]), int(metadata["width"])
    origin = np.asarray(metadata["origin_xy_m"], dtype=np.float64)
    resolution = float(metadata["resolution_m"])
    run_grid = np.zeros((height, width), dtype=np.float32)
    clearance_grid = np.zeros((height, width), dtype=np.float32)
    seed = np.zeros((height, width), dtype=np.uint8)
    strong = np.zeros((height, width), dtype=np.uint8)
    rejected_clearance = np.zeros((height, width), dtype=np.uint8)
    global_ceiling = estimate_global_ceiling(points, floor_z, voxel)
    occupied_columns = grouped_columns(points, floor_z, voxel, global_ceiling + .50)
    free_columns = grouped_columns(free_points, floor_z, voxel, global_ceiling + .20)
    for (xbin, ybin), heights in occupied_columns.items():
        xy = np.asarray([xbin * voxel, ybin * voxel])
        px = np.floor((xy - origin) / resolution).astype(int)
        if not (0 <= px[0] < width and 0 <= px[1] < height):
            continue
        local_ceiling = local_ceiling_for((xbin, ybin), occupied_columns, global_ceiling, voxel)
        connected_top = low_connected_top(heights, voxel)
        if connected_top is None:
            continue
        span_m = connected_top - min(float(heights[0]) * voxel, .55)
        clearance = max(0.0, local_ceiling - connected_top)
        run_grid[px[1], px[0]] = max(run_grid[px[1], px[0]], span_m)
        clearance_grid[px[1], px[0]] = clearance
        free_heights = free_columns.get((xbin, ybin), np.empty(0, dtype=np.int16))
        free_relative = free_heights.astype(np.float32) * voxel
        free_above = int(np.count_nonzero(
            (free_relative >= connected_top + .15)
            & (free_relative <= local_ceiling - .15)
        ))
        reaches_ceiling = clearance <= .40
        observed_clearance = clearance >= .45 and free_above >= 2
        if observed_clearance:
            rejected_clearance[px[1], px[0]] = 1
        if reaches_ceiling and span_m >= minimum_span and len(heights) >= 6:
            seed[px[1], px[0]] = 1
        if clearance <= .25 and span_m >= 1.60 and len(heights) >= 9:
            strong[px[1], px[0]] = 1

    # Source PCD columns are 0.10 m apart while the room grid is 0.05 m.
    # A five-cell connector bridges adjacent source columns before component
    # filtering; otherwise real walls appear as unrelated 1-pixel islands.
    connector = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    finish_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    connected = cv2.dilate(seed, connector, iterations=1)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(connected, 8)
    keep = np.zeros_like(seed)
    retained_components = 0
    for label in range(1, count):
        component = labels == label
        area_m2 = int(stats[label, cv2.CC_STAT_AREA]) * resolution * resolution
        component_strong = bool(np.any(strong & component))
        length_m = max(
            int(stats[label, cv2.CC_STAT_WIDTH]), int(stats[label, cv2.CC_STAT_HEIGHT])
        ) * resolution
        if area_m2 >= 0.025 and (length_m >= 0.40 or component_strong):
            keep[component] = 1
            retained_components += 1
    wall = cv2.morphologyEx(keep, cv2.MORPH_CLOSE, finish_kernel, iterations=1).astype(bool)
    return (
        wall, run_grid, clearance_grid, rejected_clearance.astype(bool),
        retained_components, int(seed.sum()), global_ceiling,
    )


def preview_grid(grid: np.ndarray) -> np.ndarray:
    image = np.zeros((*grid.shape, 3), dtype=np.uint8)
    image[grid < 0] = (155, 155, 155)
    image[grid == 0] = (248, 248, 248)
    image[grid == 100] = (24, 24, 24)
    return image


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image
    parser = argparse.ArgumentParser()
    parser.add_argument("occupied_pcd", type=Path)
    parser.add_argument("free_pcd", type=Path)
    parser.add_argument("floor_grid_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-floor", type=int, default=3)
    parser.add_argument("--voxel", type=float, default=.10)
    parser.add_argument("--minimum-vertical-span", type=float, default=1.00)
    args = parser.parse_args()

    points = load_ascii_pcd(args.occupied_pcd)
    free_points = load_ascii_pcd(args.free_pcd)
    floors = json.loads((args.floor_grid_dir / "floors.json").read_text())[:args.max_floor]
    args.output.mkdir(parents=True, exist_ok=True)
    summaries = []
    figures = []
    for item in floors:
        floor = int(item["floor"])
        floor_z = float(item["floor_z_m"])
        prefix = args.floor_grid_dir / f"L{floor}_navigation_grid"
        navigation = np.load(prefix.with_suffix(".npy"))
        metadata = json.loads(prefix.with_suffix(".json").read_text())
        wall, run_grid, clearance_grid, rejected_clearance, components, seed_cells, ceiling = extract_wall_mask(
            points, free_points, floor_z, metadata, args.voxel, args.minimum_vertical_span
        )
        structure = navigation.copy()
        structure[structure == 100] = 0
        structure[wall] = 100
        out_prefix = args.output / f"L{floor}_wall_grid"
        np.save(out_prefix.with_suffix(".npy"), structure)
        np.save(args.output / f"L{floor}_vertical_run.npy", run_grid)
        np.save(args.output / f"L{floor}_top_clearance.npy", clearance_grid)
        output_metadata = {
            **metadata,
            "format": "pre_map_vln.geometry_wall_grid.v1",
            "map_role": "geometry_extracted_room_structure",
            "floor": floor,
            "floor_z_m": floor_z,
            "wall_extraction": {
                "source_pcd": str(args.occupied_pcd),
                "source_free_pcd": str(args.free_pcd),
                "uses_truth": False,
                "uses_object_boxes": False,
                "column_voxel_m": args.voxel,
                "height_band_m": [floor_z + .15, floor_z + ceiling + .50],
                "minimum_vertical_span_m": args.minimum_vertical_span,
                "estimated_ceiling_above_floor_m": ceiling,
                "maximum_wall_top_clearance_m": .40,
                "free_clearance_rejected_cell_count": int(rejected_clearance.sum()),
                "seed_cell_count": seed_cells,
                "wall_cell_count": int(wall.sum()),
                "retained_component_count": components,
            },
        }
        out_prefix.with_suffix(".json").write_text(json.dumps(output_metadata, indent=2) + "\n")
        Image.fromarray(np.flipud(preview_grid(structure))).save(out_prefix.with_suffix(".png"))

        floor_points = points[(points[:,2] >= floor_z + .15) & (points[:,2] <= floor_z + 1.80)]
        origin = np.asarray(metadata["origin_xy_m"]); resolution=float(metadata["resolution_m"])
        extent=[origin[0],origin[0]+navigation.shape[1]*resolution,origin[1],origin[1]+navigation.shape[0]*resolution]
        fig, axes = plt.subplots(1, 3, figsize=(17, 6), sharex=True, sharey=True)
        axes[0].scatter(floor_points[:,0], floor_points[:,1], s=.35, c="#666", alpha=.35, linewidths=0, rasterized=True)
        axes[0].set_title(f"Raw occupied projection | L{floor}")
        evidence=axes[1].imshow(run_grid, origin="lower", extent=extent, cmap="magma", vmin=0, vmax=1.6, interpolation="nearest")
        axes[1].contour(wall.astype(np.uint8), levels=[.5], origin="lower", extent=extent, colors="#00e5ff", linewidths=.7)
        rejected_y, rejected_x = np.nonzero(rejected_clearance)
        axes[1].scatter(
            origin[0] + (rejected_x + .5) * resolution,
            origin[1] + (rejected_y + .5) * resolution,
            s=1.0, c="#ff4d6d", alpha=.75, linewidths=0,
        )
        axes[1].set_title("Floor-to-ceiling support\ncyan wall / pink free-above rejection")
        fig.colorbar(evidence, ax=axes[1], fraction=.045, pad=.025, label="occupied z span (m)")
        axes[2].imshow(preview_grid(structure), origin="lower", extent=extent, interpolation="nearest")
        axes[2].set_title("Geometry-only structure grid")
        for axis in axes:
            axis.set_aspect("equal"); axis.grid(alpha=.10); axis.set_xlabel("FALCON X (m)")
        axes[0].set_ylabel("FALCON Y (m)")
        fig.suptitle(f"Geometry wall extraction | L{floor} floor z={floor_z:.2f} m", y=.98)
        fig.tight_layout(rect=(0, 0, 1, .94))
        figure = args.output / f"L{floor}_wall_diagnostic.png"
        fig.savefig(figure, dpi=180, bbox_inches="tight"); plt.close(fig)
        figures.append(figure)
        summaries.append(output_metadata["wall_extraction"] | {"floor": floor})

    # Stack diagnostics without altering the original per-floor resolution.
    images = [Image.open(path).convert("RGB") for path in figures]
    width = max(image.width for image in images)
    canvas = Image.new("RGB", (width, sum(image.height for image in images)), "white")
    y = 0
    for image in images:
        canvas.paste(image, ((width - image.width)//2, y)); y += image.height
    canvas.save(args.output / "summary.png")
    (args.output / "summary.json").write_text(json.dumps({"floors": summaries}, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "floors": summaries}, indent=2))


if __name__ == "__main__":
    main()
