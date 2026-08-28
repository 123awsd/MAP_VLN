#!/usr/bin/env python3
"""Save per-scene Habitat-truth and built-map comparison figures.

The truth masks are the Habitat navmesh floor rasters already produced by the
offline evaluator.  This script only visualizes those masks alongside the
recorded observed grid; it never changes a planner input or a source Bag.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.patches import Patch
import numpy as np


SCENE_IDS = (
    "00033-oPj9qMxrDEa",
    "00062-ACZZiU6BXLz",
    "00087-YY8rqV6L6rf",
    "00108-oStKKWkQ1id",
    "00150-LcAd9dhvVwh",
    "00166-RaYrxWt5pR1",
    "00299-bdp1XNEdvmW",
)


def rgba_map(grid: np.ndarray) -> np.ndarray:
    """Render the built map: free white, occupied black, unknown gray."""
    image = np.empty((*grid.shape, 4), dtype=np.float64)
    image[...] = to_rgba("#bdbdbd")
    image[grid == 0] = to_rgba("#f7f7f7")
    image[grid == 100] = to_rgba("#222222")
    return image


def rgba_truth(mask: np.ndarray) -> np.ndarray:
    image = np.empty((*mask.shape, 4), dtype=np.float64)
    image[...] = to_rgba("#f2f2f2")
    image[mask] = to_rgba("#1976d2")
    return image


def rgba_agreement(mask: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """Render only the truth footprint, split by the built-map state."""
    image = np.empty((*grid.shape, 4), dtype=np.float64)
    image[...] = to_rgba("#f2f2f2")
    image[mask & (grid == 0)] = to_rgba("#43a047")
    image[mask & (grid == 100)] = to_rgba("#ef6c00")
    image[mask & (grid < 0)] = to_rgba("#757575")
    return image


def rgba_all_floors(floor_targets: np.ndarray) -> np.ndarray:
    image = np.empty((*floor_targets.shape[1:], 4), dtype=np.float64)
    image[...] = to_rgba("#f2f2f2")
    palette = [
        "#1565c0", "#6a1b9a", "#00838f", "#2e7d32",
        "#ef6c00", "#ad1457", "#5d4037", "#455a64",
    ]
    for index, mask in enumerate(floor_targets):
        color = np.asarray(to_rgba(palette[index % len(palette)], alpha=0.70))
        existing = image[mask]
        image[mask, :3] = color[3] * color[:3] + (1.0 - color[3]) * existing[:, :3]
        image[mask, 3] = 1.0
    return image


def extent(meta: dict, shape: tuple[int, int]) -> list[float]:
    origin = np.asarray(meta["origin_xy_m"], dtype=np.float64)
    resolution = float(meta["resolution_m"])
    height, width = shape
    return [
        float(origin[0]),
        float(origin[0] + width * resolution),
        float(origin[1]),
        float(origin[1] + height * resolution),
    ]


def draw_panel(ax, image: np.ndarray, meta: dict, title: str) -> None:
    ax.imshow(
        image,
        origin="lower",
        extent=extent(meta, image.shape[:2]),
        interpolation="nearest",
        aspect="equal",
    )
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("FALCON x (m)")
    ax.set_ylabel("FALCON y (m)")
    ax.grid(False)


def save_figure(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def scene_plot(batch_root: Path, scene_id: str) -> dict:
    run_dir = batch_root / scene_id
    metrics = json.loads((run_dir / "truth_metrics.json").read_text(encoding="utf-8"))
    meta = json.loads((run_dir / "observed_ground.json").read_text(encoding="utf-8"))
    arrays = np.load(run_dir / "truth_metrics.npz")
    grid = np.asarray(arrays["observed_grid"])
    floor_targets = np.asarray(arrays["floor_targets"], dtype=bool)
    if floor_targets.ndim != 3 or floor_targets.shape[1:] != grid.shape:
        raise ValueError(f"truth/grid shape mismatch for {scene_id}")
    selection = metrics["truth"]["executed_floor_selection"]
    executed_index = selection["nearest_survey_floor_index"]
    if executed_index is None or not 0 <= int(executed_index) < len(floor_targets):
        executed_index = 0
    executed_index = int(executed_index)
    target = floor_targets[executed_index]
    values = grid[target]
    known = int(np.count_nonzero(values >= 0))
    free = int(np.count_nonzero(values == 0))
    occupied = int(np.count_nonzero(values == 100))
    unknown = int(np.count_nonzero(values < 0))
    target_cells = int(len(values))

    out_dir = run_dir / "comparison"
    out_dir.mkdir(parents=True, exist_ok=True)
    truth_path = out_dir / "truth_executed_floor.png"
    map_path = out_dir / "map_observed_ground.png"
    all_floors_path = out_dir / "truth_all_floors.png"
    comparison_path = out_dir / "truth_vs_map.png"
    data_path = out_dir / "truth_and_observed.npz"

    fig, ax = plt.subplots(figsize=(8, 7))
    draw_panel(
        ax, rgba_truth(target), meta,
        f"{scene_id} | Habitat navmesh truth | floor {executed_index}",
    )
    save_figure(fig, truth_path)

    fig, ax = plt.subplots(figsize=(8, 7))
    draw_panel(ax, rgba_map(grid), meta, f"{scene_id} | built observed ground map")
    ax.legend(handles=[
        Patch(facecolor="#f7f7f7", edgecolor="#777", label="observed free"),
        Patch(facecolor="#222222", label="observed occupied"),
        Patch(facecolor="#bdbdbd", label="unknown"),
    ], loc="upper right", fontsize=8, framealpha=0.9)
    save_figure(fig, map_path)

    fig, ax = plt.subplots(figsize=(8, 7))
    draw_panel(ax, rgba_all_floors(floor_targets), meta, f"{scene_id} | all Habitat floor truth masks")
    handles = [
        Patch(facecolor="#1565c0", alpha=0.70, label=f"floor {index}")
        for index in range(min(len(floor_targets), 8))
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.9)
    save_figure(fig, all_floors_path)

    fig, axes = plt.subplots(2, 2, figsize=(15, 12), constrained_layout=True)
    draw_panel(
        axes[0, 0], rgba_truth(target), meta,
        f"Habitat truth: executed floor {executed_index}",
    )
    draw_panel(axes[0, 1], rgba_map(grid), meta, "Built map: free / occupied / unknown")
    draw_panel(
        axes[1, 0], rgba_agreement(target, grid), meta,
        f"Truth vs built map | known {known / target_cells:.1%} | free {free / target_cells:.1%}",
    )
    draw_panel(axes[1, 1], rgba_all_floors(floor_targets), meta, "All Habitat floor truth masks")
    axes[0, 1].legend(handles=[
        Patch(facecolor="#f7f7f7", edgecolor="#777", label="built free"),
        Patch(facecolor="#222222", label="built occupied"),
        Patch(facecolor="#bdbdbd", label="built unknown"),
    ], loc="upper right", fontsize=8, framealpha=0.9)
    axes[1, 0].legend(handles=[
        Patch(facecolor="#43a047", label="truth + built free"),
        Patch(facecolor="#ef6c00", label="truth + built occupied"),
        Patch(facecolor="#757575", label="truth + unknown"),
    ], loc="upper right", fontsize=8, framealpha=0.9)
    fig.suptitle(
        f"{scene_id} | stage1 truth / built-map comparison | resolution {meta['resolution_m']} m",
        fontsize=14,
    )
    save_figure(fig, comparison_path)

    np.savez_compressed(
        data_path,
        observed_grid=grid,
        executed_floor_truth=target,
        all_floor_truth=floor_targets,
    )
    summary = {
        "format": "pre_map_vln.hm3d_stage1_comparison.v1",
        "scene": scene_id,
        "source_truth_metrics": str(run_dir / "truth_metrics.json"),
        "source_observed_grid": str(run_dir / "observed_ground.npy"),
        "executed_floor_index": executed_index,
        "executed_floor_height_m": selection["nearest_survey_floor_height_m"],
        "target_cells": target_cells,
        "known_cells": known,
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "known_fraction": None if not target_cells else known / target_cells,
        "strict_free_fraction": None if not target_cells else free / target_cells,
        "unknown_fraction": None if not target_cells else unknown / target_cells,
        "files": {
            "truth_executed_floor_png": str(truth_path),
            "map_observed_ground_png": str(map_path),
            "truth_all_floors_png": str(all_floors_path),
            "truth_vs_map_png": str(comparison_path),
            "truth_and_observed_npz": str(data_path),
        },
    }
    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch-root", type=Path, default=Path("outputs/stage1_seven_v6"))
    parser.add_argument("--scene", action="append", choices=SCENE_IDS)
    args = parser.parse_args()
    scenes = args.scene or list(SCENE_IDS)
    summaries = [scene_plot(args.batch_root, scene_id) for scene_id in scenes]
    index = {
        "format": "pre_map_vln.hm3d_stage1_comparison_index.v1",
        "batch_root": str(args.batch_root),
        "scenes": summaries,
    }
    index_path = args.batch_root / "comparison_index.json"
    index_path.write_text(
        json.dumps(index, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(index, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
