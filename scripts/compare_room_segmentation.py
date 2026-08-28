#!/usr/bin/env python3
"""Compare Habitat navmesh truth, explored occupancy, and canonical room boundaries."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/pre_map_vln_matplotlib")

import habitat_sim
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=ROOT / "data/scene_datasets/hm3d/example/00861-GLAQ4DNUx5U/GLAQ4DNUx5U.basis.glb")
    parser.add_argument("--scene-config", type=Path, default=ROOT / "data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json")
    parser.add_argument("--grid-prefix", type=Path, default=ROOT / "runtime/occusg/hm3d_stage1_complete_v3_structure_grid")
    parser.add_argument("--scene-graph", type=Path, default=ROOT / "outputs/scene_graph/hm3d_stage1_complete_v3.json")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/room_comparison/hm3d_stage1_complete_v3.png")
    args = parser.parse_args()

    grid_meta = json.loads(args.grid_prefix.with_suffix(".json").read_text())
    grid = np.load(args.grid_prefix.with_suffix(".npy"))
    graph = json.loads(args.scene_graph.read_text())
    resolution = float(grid_meta["resolution_m"])
    scene_label = args.scene.parent.name

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(args.scene)
    sim_cfg.scene_dataset_config_file = str(args.scene_config)
    sim_cfg.create_renderer = False
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("Habitat navmesh did not load")
        sim.pathfinder.seed(7)
        origin_h = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float64)
        lower_h, upper_h = (np.asarray(value, dtype=np.float64) for value in sim.pathfinder.get_bounds())
        navmesh = np.asarray(
            sim.pathfinder.get_topdown_view(resolution, float(origin_h[1])), dtype=bool
        )

    rows, cols = np.nonzero(navmesh)
    habitat_x = lower_h[0] + (cols.astype(np.float64) + 0.5) * resolution
    habitat_z = lower_h[2] + (rows.astype(np.float64) + 0.5) * resolution
    nav_x = -(habitat_z - origin_h[2])
    nav_y = -(habitat_x - origin_h[0])

    grid_origin = np.asarray(grid_meta["origin_xy_m"], dtype=np.float64)
    grid_extent = [
        grid_origin[0], grid_origin[0] + grid.shape[1] * resolution,
        grid_origin[1], grid_origin[1] + grid.shape[0] * resolution,
    ]
    display_grid = np.zeros_like(grid, dtype=np.uint8)
    display_grid[grid < 0] = 0
    display_grid[grid == 0] = 1
    display_grid[grid > 0] = 2
    occupancy_cmap = ListedColormap(["#d9d9d9", "#ffffff", "#202020"])

    fig, axes = plt.subplots(1, 3, figsize=(18, 7), sharex=True, sharey=True)
    axes[0].scatter(nav_x, nav_y, s=1.2, c="#5f6770", marker="s", linewidths=0)
    axes[0].set_title("Habitat navmesh truth\n(navigable floor, not room labels)")
    axes[1].imshow(
        display_grid, origin="lower", extent=grid_extent, interpolation="nearest",
        cmap=occupancy_cmap, vmin=0, vmax=2,
    )
    map_title = "Room structure occupancy" if grid_meta.get("map_role") == "room_structure" else "Navigation occupancy"
    axes[1].set_title(f"{map_title}\n(gray unknown / white free / black occupied)")
    axes[2].scatter(nav_x, nav_y, s=1.2, c="#d7dce0", marker="s", linewidths=0)
    axes[2].set_title("Canonical room segmentation\nover official navmesh")

    exclusions = grid_meta.get("single_floor_policy", {}).get("excluded_navigation_regions", [])
    for axis in axes[1:]:
        for exclusion in exclusions:
            axis.add_patch(Polygon(
                exclusion["polygon_xy_m"], closed=True, facecolor="#d94841",
                edgecolor="#a61b1b", linewidth=1.2, alpha=0.22, hatch="///",
            ))
    if exclusions:
        polygon = np.asarray(exclusions[0]["polygon_xy_m"], dtype=np.float64)
        center = polygon.mean(axis=0)
        axes[2].text(
            center[0], center[1], "STAIRS EXCLUDED", fontsize=8, weight="bold",
            color="#8d1515", ha="center", va="center", rotation=22,
        )

    colors = {
        "bedroom": "#6b4ec4", "living_room": "#008b9a", "kitchen": "#e67e22",
        "bathroom": "#2878c8", "unknown": "#56616a", "corridor": "#8a8a8a",
    }
    for room in graph["rooms"]:
        polygon = room["polygon_xy_m"]
        if not polygon:
            continue
        points = polygon + polygon[:1]
        x, y = zip(*points)
        room_type = str(room.get("semantic_type", "unknown"))
        color = colors.get(room_type, colors["unknown"])
        linewidth = 1.5 if room.get("space_role") == "transition_space" else 2.8
        axes[2].plot(x, y, color=color, linewidth=linewidth)
        centroid = room["centroid_xy_m"]
        axes[2].text(
            centroid[0], centroid[1], f"R{room['id']} {room_type}",
            fontsize=8, ha="center", va="center", color="#202020",
            bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": color, "alpha": 0.82},
        )

    all_x = np.concatenate([nav_x, np.asarray([grid_extent[0], grid_extent[1]])])
    all_y = np.concatenate([nav_y, np.asarray([grid_extent[2], grid_extent[3]])])
    margin = 0.5
    for axis in axes:
        axis.set_aspect("equal")
        axis.set_xlim(float(all_x.min() - margin), float(all_x.max() + margin))
        axis.set_ylim(float(all_y.min() - margin), float(all_y.max() + margin))
        axis.set_xlabel("Falcon X (m)")
        axis.grid(alpha=0.12)
    axes[0].set_ylabel("Falcon Y (m)")
    fig.suptitle(f"HM3D {scene_label} room-boundary diagnosis", fontsize=16)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight")
    report = {
        "format": "pre_map_vln.room_comparison.v1",
        "scene": args.scene.parent.name,
        "floor_height_habitat_m": float(origin_h[1]),
        "habitat_origin_xz_m": [float(origin_h[0]), float(origin_h[2])],
        "resolution_m": resolution,
        "navmesh_navigable_cells": int(np.count_nonzero(navmesh)),
        "canonical_region_count": len(graph["rooms"]),
        "canonical_room_count": sum(room.get("space_role") == "room" for room in graph["rooms"]),
        "corridor_count": sum(room.get("space_role") == "transition_space" for room in graph["rooms"]),
        "source_fragment_count": int(graph["summary"].get("source_fragment_count", 0)),
        "stairs_excluded": bool(exclusions),
        "navigation_exclusion_cell_count": int(
            grid_meta.get("single_floor_policy", {}).get("excluded_cell_count", 0)
        ),
        "note": "Navmesh is official navigability geometry, not human-authored room segmentation.",
    }
    args.output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
