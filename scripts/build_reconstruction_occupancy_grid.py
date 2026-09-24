#!/usr/bin/env python3
"""Build a ROS occupancy grid from the cleaned, roofless real-scene PLY."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image
from PIL import ImageDraw
from scipy import ndimage
from scipy.spatial import ConvexHull

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


PLY_TYPES = {
    "char": "i1",
    "uchar": "u1",
    "short": "<i2",
    "ushort": "<u2",
    "int": "<i4",
    "uint": "<u4",
    "float": "<f4",
    "double": "<f8",
}


def read_binary_ply(path: Path) -> np.ndarray:
    properties: list[tuple[str, str]] = []
    vertex_count = None
    with path.open("rb") as stream:
        if stream.readline() != b"ply\n":
            raise ValueError(f"Not a PLY file: {path}")
        if stream.readline().strip() != b"format binary_little_endian 1.0":
            raise ValueError(f"Only binary little-endian PLY is supported: {path}")
        while True:
            words = stream.readline().decode("ascii").strip().split()
            if words[:2] == ["element", "vertex"]:
                vertex_count = int(words[2])
            elif words[:1] == ["property"] and len(words) == 3:
                properties.append((words[2], PLY_TYPES[words[1]]))
            elif words[:1] == ["end_header"]:
                offset = stream.tell()
                break
    if vertex_count is None:
        raise ValueError(f"PLY has no vertex element: {path}")
    required = {"x", "y", "z", "label"}
    if not required <= {name for name, _ in properties}:
        raise ValueError(f"PLY must contain {sorted(required)}: {path}")
    return np.memmap(
        path, dtype=np.dtype(properties), mode="r", offset=offset, shape=(vertex_count,)
    )


def remove_small_components(mask: np.ndarray, minimum_cells: int) -> np.ndarray:
    components, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
    if not count:
        return mask
    sizes = np.bincount(components.ravel())
    keep = sizes >= minimum_cells
    keep[0] = False
    return keep[components]


def fill_compact_room_gaps(
    free: np.ndarray, occupied: np.ndarray
) -> tuple[np.ndarray, int]:
    """Fill unobserved gaps inside compact, separately observed small rooms."""
    components, count = ndimage.label(free, structure=np.ones((3, 3), dtype=bool))
    sizes = np.bincount(components.ravel())
    result = free.copy()
    filled_cells = 0
    height, width = free.shape
    for component in range(1, count + 1):
        size = sizes[component]
        if not 300 <= size <= 2_000:
            continue
        component_mask = components == component
        # Nearby wall cells reveal the missing side of a partly observed room.
        # Include them when estimating the room outline, but never mark the
        # wall cells themselves as free.
        boundary_support = component_mask | (
            ndimage.binary_dilation(component_mask, iterations=5) & occupied
        )
        y, x = np.nonzero(boundary_support)
        try:
            hull = ConvexHull(np.column_stack((x, y)))
        except ValueError:
            continue
        polygon = [(int(x[index]), int(y[index])) for index in hull.vertices]
        image = Image.new("1", (width, height), 0)
        ImageDraw.Draw(image).polygon(polygon, fill=1)
        candidate = np.asarray(image, dtype=bool) & ~occupied
        added = candidate & ~result
        # Do not convexify strongly non-convex regions masquerading as rooms.
        if added.sum() <= 0.8 * size:
            result |= candidate
            filled_cells += int(added.sum())
    return result, filled_cells


def make_grid(
    points: np.ndarray, resolution: float, margin: float
) -> tuple[np.ndarray, dict[str, float | int], dict[str, int]]:
    min_x = math.floor((float(points["x"].min()) - margin) / resolution) * resolution
    min_y = math.floor((float(points["y"].min()) - margin) / resolution) * resolution
    max_x = math.ceil((float(points["x"].max()) + margin) / resolution) * resolution
    max_y = math.ceil((float(points["y"].max()) + margin) / resolution) * resolution
    width = int(round((max_x - min_x) / resolution))
    height = int(round((max_y - min_y) / resolution))

    cell_x = np.floor((points["x"] - min_x) / resolution).astype(np.int32)
    cell_y = np.floor((points["y"] - min_y) / resolution).astype(np.int32)
    inside = (cell_x >= 0) & (cell_x < width) & (cell_y >= 0) & (cell_y < height)
    flat = cell_y[inside] * width + cell_x[inside]
    labels = points["label"][inside]
    z = np.asarray(points["z"][inside])
    cells = width * height

    wall_points = labels == 2
    wall_flat = flat[wall_points]
    wall_z = z[wall_points]
    wall_count = np.bincount(wall_flat, minlength=cells)
    wall_min = np.full(cells, np.inf, dtype=np.float32)
    wall_max = np.full(cells, -np.inf, dtype=np.float32)
    np.minimum.at(wall_min, wall_flat, wall_z)
    np.maximum.at(wall_max, wall_flat, wall_z)
    wall_span = wall_max - wall_min

    # A true wall occupies several height bins.  Requiring vertical support
    # rejects furniture surfaces and the sparse points that survived 3-D cleanup.
    wall_core = ((wall_count >= 3) & (wall_span >= 0.55)).reshape(height, width)
    wall_support = ((wall_count >= 2) & (wall_span >= 0.20)).reshape(height, width)
    occupied = wall_core | (ndimage.binary_dilation(wall_core, iterations=1) & wall_support)
    occupied = ndimage.binary_closing(occupied, structure=np.ones((3, 3), dtype=bool))
    occupied = remove_small_components(occupied, minimum_cells=6)
    occupied = ndimage.binary_dilation(occupied, iterations=1)

    ground_flat = flat[labels == 1]
    ground_count = np.bincount(ground_flat, minlength=cells).reshape(height, width)
    free = ground_count > 0
    free = ndimage.binary_dilation(free, iterations=2)
    free = ndimage.binary_closing(free, structure=np.ones((5, 5), dtype=bool))
    free = remove_small_components(free, minimum_cells=80)
    # Retain walls only when they border a supported floor observation.  This
    # removes remote wall-like fragments without erasing room boundaries.
    occupied &= ndimage.binary_dilation(free, iterations=4)
    occupied = remove_small_components(occupied, minimum_cells=6)
    free &= ~occupied
    free, room_gap_cells_filled = fill_compact_room_gaps(free, occupied)

    # Internal representation follows nav_msgs/OccupancyGrid: -1 unknown,
    # 0 free, 100 occupied.  Array row zero corresponds to world min_y.
    grid = np.full((height, width), -1, dtype=np.int8)
    grid[free] = 0
    grid[occupied] = 100
    metadata: dict[str, float | int] = {
        "resolution": resolution,
        "origin_x": min_x,
        "origin_y": min_y,
        "origin_yaw": 0.0,
        "width": width,
        "height": height,
    }
    statistics = {
        "occupied_cells": int(occupied.sum()),
        "free_cells": int(free.sum()),
        "unknown_cells": int((grid < 0).sum()),
        "room_floor_gap_cells_filled": room_gap_cells_filled,
    }
    return grid, metadata, statistics


def save_ros_map(output_dir: Path, grid: np.ndarray, metadata: dict[str, float | int]) -> None:
    # map_server convention: black=occupied, white=free, gray=unknown; image
    # origin is top-left, so flip world-Y before writing.
    pgm = np.full(grid.shape, 205, dtype=np.uint8)
    pgm[grid == 0] = 254
    pgm[grid == 100] = 0
    Image.fromarray(np.flipud(pgm), mode="L").save(output_dir / "occupancy_grid.pgm")
    yaml_text = (
        "image: occupancy_grid.pgm\n"
        f"resolution: {metadata['resolution']:.6f}\n"
        f"origin: [{metadata['origin_x']:.6f}, {metadata['origin_y']:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    (output_dir / "occupancy_grid.yaml").write_text(yaml_text)
    np.savez_compressed(
        output_dir / "occupancy_grid.npz",
        occupancy=grid,
        resolution=np.float32(metadata["resolution"]),
        origin=np.array([metadata["origin_x"], metadata["origin_y"], 0.0]),
    )


def save_preview(
    output_dir: Path, grid: np.ndarray, metadata: dict[str, float | int]
) -> None:
    preview = np.full(grid.shape, 1, dtype=np.uint8)  # unknown
    preview[grid == 0] = 2
    preview[grid == 100] = 0
    extent = [
        metadata["origin_x"],
        metadata["origin_x"] + metadata["width"] * metadata["resolution"],
        metadata["origin_y"],
        metadata["origin_y"] + metadata["height"] * metadata["resolution"],
    ]
    figure, axis = plt.subplots(figsize=(10, 10), dpi=180)
    axis.imshow(
        preview,
        origin="lower",
        extent=extent,
        interpolation="nearest",
        cmap=ListedColormap(["#17191d", "#878b91", "#f4f4f1"]),
        vmin=0,
        vmax=2,
    )
    axis.set_aspect("equal")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_title("Clean roofless occupancy grid (black=occupied, white=free)")
    axis.grid(color="white", alpha=0.12, linewidth=0.4)
    figure.tight_layout()
    figure.savefig(output_dir / "occupancy_grid_preview.png", bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_ply", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--margin", type=float, default=1.0)
    args = parser.parse_args()
    if args.resolution <= 0 or args.margin < 0:
        parser.error("resolution must be positive and margin must be non-negative")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    points = read_binary_ply(args.input_ply)
    grid, metadata, statistics = make_grid(points, args.resolution, args.margin)
    save_ros_map(args.output_dir, grid, metadata)
    save_preview(args.output_dir, grid, metadata)
    report = {
        "source_ply": str(args.input_ply),
        "source_points": len(points),
        **metadata,
        **statistics,
        "cleaning": {
            "roof_and_overhead_removed_upstream": True,
            "wall_min_points_per_cell": 3,
            "wall_min_vertical_span_m": 0.55,
            "small_occupied_component_min_cells": 6,
            "small_free_component_min_cells": 80,
            "wall_max_distance_from_supported_floor_m": 0.4,
        },
        "outputs": {
            "ros_map_yaml": str(args.output_dir / "occupancy_grid.yaml"),
            "ros_map_pgm": str(args.output_dir / "occupancy_grid.pgm"),
            "numpy_grid": str(args.output_dir / "occupancy_grid.npz"),
            "preview": str(args.output_dir / "occupancy_grid_preview.png"),
        },
    }
    (args.output_dir / "occupancy_grid_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
