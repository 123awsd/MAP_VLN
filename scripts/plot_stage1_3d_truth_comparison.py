#!/usr/bin/env python3
"""Plot a FALCON 3-D occupancy map against the official HM3D basis mesh.

The figure is a geometric visualization, not an area/volume coverage metric.
HM3D mesh vertices are transformed into the exact FALCON frame recorded in
the Habitat episode manifest before they are overlaid with the final occupied
PointCloud2 export and executed sensor trajectory.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import habitat_sim
import magnum.trade as trade
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
import numpy as np
from scipy.ndimage import binary_dilation


ROOT = Path(__file__).resolve().parents[1]
FALCON_SENSOR_ORIGIN = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)


def resolve_scene(path: str) -> Path:
    scene = Path(path)
    if not scene.is_absolute():
        scene = ROOT / scene
    if not scene.is_file():
        raise FileNotFoundError(f"HM3D scene not found: {scene}")
    return scene.resolve()


def resolve_asset(path: str) -> Path:
    asset = Path(path)
    if not asset.is_absolute():
        asset = ROOT / asset
    if not asset.is_file():
        raise FileNotFoundError(f"Asset not found: {asset}")
    return asset.resolve()


def load_hm3d_vertices(scene: Path, manifest: dict) -> np.ndarray:
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    if importer is None:
        raise RuntimeError("Magnum AnySceneImporter is unavailable")
    importer.open_file(str(scene))

    chunks = []
    for index in range(importer.mesh_count):
        mesh = importer.mesh(index)
        if not mesh.has_attribute(trade.MeshAttribute.POSITION):
            continue
        chunks.append(
            np.asarray(mesh.attribute(trade.MeshAttribute.POSITION), dtype=np.float64)
        )
    if not chunks:
        raise RuntimeError(f"No mesh vertices found in {scene}")
    vertices_basis = np.concatenate(chunks, axis=0)

    # HM3D basis GLBs use z-up coordinates. Habitat exposes the same geometry
    # as y-up: (basis x, basis y, basis z) -> (x, z, -y).
    vertices_habitat = np.column_stack(
        (vertices_basis[:, 0], vertices_basis[:, 2], -vertices_basis[:, 1])
    )
    initial_sensor = np.asarray(
        manifest["initial_sensor_habitat_xyz"], dtype=np.float64
    )
    transform = np.asarray(
        manifest["main_floor_mapping"]["transform"], dtype=np.float64
    )
    return (
        (transform @ (vertices_habitat - initial_sensor).T).T
        + FALCON_SENSOR_ORIGIN
    )


def detect_mesh_floor_planes(scene: Path, manifest: dict) -> list[dict]:
    """Find large upward-facing horizontal planes in the official HM3D mesh."""
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    if importer is None:
        raise RuntimeError("Magnum AnySceneImporter is unavailable")
    importer.open_file(str(scene))
    initial_sensor = np.asarray(
        manifest["initial_sensor_habitat_xyz"], dtype=np.float64
    )
    transform = np.asarray(
        manifest["main_floor_mapping"]["transform"], dtype=np.float64
    )
    area_by_height: dict[float, float] = {}
    for index in range(importer.mesh_count):
        mesh = importer.mesh(index)
        if not mesh.has_attribute(trade.MeshAttribute.POSITION):
            continue
        vertices_basis = np.asarray(
            mesh.attribute(trade.MeshAttribute.POSITION), dtype=np.float64
        )
        vertices_habitat = np.column_stack(
            (vertices_basis[:, 0], vertices_basis[:, 2], -vertices_basis[:, 1])
        )
        vertices = (
            (transform @ (vertices_habitat - initial_sensor).T).T
            + FALCON_SENSOR_ORIGIN
        )
        indices = (
            np.asarray(mesh.indices, dtype=np.int64)
            if mesh.is_indexed else np.arange(len(vertices), dtype=np.int64)
        )
        if len(indices) % 3:
            continue
        triangles = vertices[indices.reshape(-1, 3)]
        cross = np.cross(
            triangles[:, 1] - triangles[:, 0],
            triangles[:, 2] - triangles[:, 0],
        )
        doubled_area = np.linalg.norm(cross, axis=1)
        normal_z = np.divide(
            cross[:, 2], doubled_area,
            out=np.zeros_like(doubled_area), where=doubled_area > 1e-9,
        )
        areas = doubled_area * 0.5
        keep = (normal_z >= 0.94) & (areas > 1e-6)
        heights = np.round(triangles[keep, :, 2].mean(axis=1) / 0.05) * 0.05
        for height, area in zip(heights, areas[keep]):
            key = float(height)
            area_by_height[key] = area_by_height.get(key, 0.0) + float(area)

    if not area_by_height:
        return []
    max_area = max(area_by_height.values())
    minimum_area = max(3.0, 0.03 * max_area)
    planes = []
    for height, area in sorted(
        area_by_height.items(), key=lambda item: item[1], reverse=True
    ):
        if area < minimum_area:
            continue
        if any(abs(height - item["falcon_floor_z_m"]) < 0.30 for item in planes):
            continue
        planes.append({
            "falcon_floor_z_m": height,
            "mesh_upward_area_m2": area,
        })
    return sorted(planes, key=lambda item: item["falcon_floor_z_m"])


def load_ascii_pcd(path: Path) -> np.ndarray:
    if not path.is_file():
        raise FileNotFoundError(
            f"Final occupied PCD not found: {path}. Export the Bag first."
        )
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip().upper().startswith("DATA"):
                if line.strip().lower() != "data ascii":
                    raise ValueError(f"Only ASCII PCD is supported: {path}")
                break
        else:
            raise ValueError(f"Invalid PCD header: {path}")
        points = np.loadtxt(handle, dtype=np.float64, ndmin=2)
    if points.shape[1] < 3:
        raise ValueError(f"PCD has fewer than three fields: {path}")
    return points[:, :3]


def load_trajectory(path: Path) -> np.ndarray:
    if not path.is_file():
        return np.empty((0, 3), dtype=np.float64)
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return np.asarray(
        [[float(row[axis]) for axis in ("x", "y", "z")] for row in rows],
        dtype=np.float64,
    )


def deterministic_sample(points: np.ndarray, limit: int) -> np.ndarray:
    if len(points) <= limit:
        return points
    indices = np.linspace(0, len(points) - 1, limit, dtype=np.int64)
    return points[indices]


def detect_navmesh_floors(
    scene: Path, scene_config: Path, manifest: dict, samples: int = 50_000
) -> tuple[list[dict], list[dict]]:
    """Detect full floors from strong navmesh height peaks.

    Peaks less than 1.8 m from an accepted floor are reported as platforms,
    not independent floors. A peak must represent at least 2% of uniformly
    sampled navigable area to avoid counting tiny stair treads.
    """
    simulator_config = habitat_sim.SimulatorConfiguration()
    simulator_config.scene_id = str(scene)
    simulator_config.scene_dataset_config_file = str(scene_config)
    simulator_config.create_renderer = False
    agent_config = habitat_sim.agent.AgentConfiguration()
    with habitat_sim.Simulator(
        habitat_sim.Configuration(simulator_config, [agent_config])
    ) as simulator:
        if not simulator.pathfinder.is_loaded:
            raise RuntimeError(f"Habitat navmesh did not load: {scene}")
        simulator.pathfinder.seed(17)
        heights = np.asarray(
            [simulator.pathfinder.get_random_navigable_point()[1] for _ in range(samples)],
            dtype=np.float64,
        )

    rounded = np.round(heights / 0.1) * 0.1
    values, counts = np.unique(rounded, return_counts=True)
    candidates = [
        {"habitat_y_m": float(values[index]), "peak_fraction": float(counts[index] / samples)}
        for index in np.argsort(counts)[::-1]
        if counts[index] / samples >= 0.02
    ]
    floors = []
    platforms = []
    for candidate in candidates:
        nearest = min(
            (abs(candidate["habitat_y_m"] - item["habitat_y_m"]) for item in floors),
            default=float("inf"),
        )
        if nearest >= 1.8:
            floors.append(candidate)
        elif nearest >= 0.35:
            platforms.append(candidate)
    floors.sort(key=lambda item: item["habitat_y_m"])
    platforms.sort(key=lambda item: item["habitat_y_m"])

    initial_sensor_y = float(manifest["initial_sensor_habitat_xyz"][1])
    for item in floors + platforms:
        item["falcon_floor_z_m"] = item["habitat_y_m"] - initial_sensor_y + 1.0
    return floors, platforms


def detect_hybrid_floors(
    scene: Path,
    scene_config: Path,
    manifest: dict,
    occupied: np.ndarray,
) -> tuple[list[dict], list[dict]]:
    """Cross-check navmesh levels against official and reconstructed floors."""
    navmesh_floors, platforms = detect_navmesh_floors(
        scene, scene_config, manifest
    )
    mesh_planes = detect_mesh_floor_planes(scene, manifest)
    floors = []
    for navmesh_floor in navmesh_floors:
        navmesh_z = float(navmesh_floor["falcon_floor_z_m"])
        nearest = min(
            mesh_planes,
            key=lambda item: abs(item["falcon_floor_z_m"] - navmesh_z),
            default=None,
        )
        if nearest is not None and abs(nearest["falcon_floor_z_m"] - navmesh_z) <= 0.40:
            floor_z = float(nearest["falcon_floor_z_m"])
            source = "official_horizontal_plane+navmesh+falcon_pointcloud"
            mesh_area = float(nearest["mesh_upward_area_m2"])
        else:
            floor_z = navmesh_z
            source = "navmesh+falcon_pointcloud; no matching mesh plane"
            mesh_area = None
        support = int(np.count_nonzero(np.abs(occupied[:, 2] - floor_z) <= 0.25))
        floors.append({
            **navmesh_floor,
            "navmesh_falcon_z_m": navmesh_z,
            "falcon_floor_z_m": floor_z,
            "mesh_upward_area_m2": mesh_area,
            "falcon_floor_support_points": support,
            "detection_source": source,
        })
    return floors, platforms


def rasterize_xy(points: np.ndarray, lower: np.ndarray, shape: tuple[int, int], resolution: float) -> np.ndarray:
    mask = np.zeros(shape, dtype=bool)
    if not len(points):
        return mask
    indices = np.floor((points[:, :2] - lower) / resolution).astype(np.int64)
    valid = (
        (indices[:, 0] >= 0) & (indices[:, 0] < shape[1])
        & (indices[:, 1] >= 0) & (indices[:, 1] < shape[0])
    )
    indices = indices[valid]
    mask[indices[:, 1], indices[:, 0]] = True
    return mask


def plot_floor_projections(
    truth: np.ndarray,
    occupied: np.ndarray,
    floors: list[dict],
    platforms: list[dict],
    output: Path,
    scene_name: str,
    termination: str,
    resolution: float = 0.1,
) -> dict:
    all_points = np.concatenate((truth, occupied), axis=0)
    lower_xy = np.floor(all_points[:, :2].min(axis=0) / resolution) * resolution
    upper_xy = np.ceil(all_points[:, :2].max(axis=0) / resolution) * resolution
    width, height = np.maximum(
        np.ceil((upper_xy - lower_xy) / resolution).astype(int) + 1, 1
    )
    shape = (int(height), int(width))
    extent = [lower_xy[0], lower_xy[0] + width * resolution,
              lower_xy[1], lower_xy[1] + height * resolution]

    floor_z = np.asarray([item["falcon_floor_z_m"] for item in floors], dtype=np.float64)
    fig, axes = plt.subplots(
        len(floors), 3, figsize=(16, max(4.6 * len(floors), 5.0)), squeeze=False,
        constrained_layout=True,
    )
    floor_summaries = []
    for index, floor in enumerate(floors):
        # Project only the middle-height structure band. Including the floor
        # and ceiling surfaces would fill almost every x-y cell and conceal
        # exactly the missing walls/rooms this comparison is meant to show.
        z0 = float(floor_z[index] + 0.30)
        z1 = float(floor_z[index] + 2.30)
        if index + 1 < len(floor_z):
            z1 = min(z1, float(floor_z[index + 1] - 0.30))
        if z1 <= z0:
            z1 = z0 + 1.0
        truth_layer = truth[(truth[:, 2] >= z0) & (truth[:, 2] < z1)]
        map_layer = occupied[(occupied[:, 2] >= z0) & (occupied[:, 2] < z1)]
        truth_mask = rasterize_xy(truth_layer, lower_xy, shape, resolution)
        map_mask = rasterize_xy(map_layer, lower_xy, shape, resolution)

        # A 0.3 m tolerance absorbs voxel-center and mesh-surface sampling
        # offsets. Red therefore highlights meaningful missing structure, not
        # one-pixel discretization noise.
        tolerance_cells = max(1, int(round(0.3 / resolution)))
        dilated_map = binary_dilation(map_mask, iterations=tolerance_cells)
        dilated_truth = binary_dilation(truth_mask, iterations=tolerance_cells)
        matched_truth = truth_mask & dilated_map
        missing_truth = truth_mask & ~dilated_map
        map_only = map_mask & ~dilated_truth

        truth_image = np.zeros(shape, dtype=np.uint8)
        truth_image[truth_mask] = 1
        map_image = np.zeros(shape, dtype=np.uint8)
        map_image[map_mask] = 1
        diff_image = np.zeros(shape, dtype=np.uint8)
        diff_image[matched_truth] = 1
        diff_image[missing_truth] = 2
        diff_image[map_only] = 3

        axes[index, 0].imshow(
            truth_image, origin="lower", extent=extent, interpolation="nearest",
            cmap=ListedColormap(["#ffffff", "#111111"]), vmin=0, vmax=1,
        )
        axes[index, 1].imshow(
            map_image, origin="lower", extent=extent, interpolation="nearest",
            cmap=ListedColormap(["#ffffff", "#1976d2"]), vmin=0, vmax=1,
        )
        axes[index, 2].imshow(
            diff_image, origin="lower", extent=extent, interpolation="nearest",
            cmap=ListedColormap(["#ffffff", "#222222", "#d50000", "#ff8a80"]),
            vmin=0, vmax=3,
        )
        label = f"Level {index + 1} | floor z={floor_z[index]:.2f} m | wall band [{z0:.2f}, {z1:.2f}) m"
        axes[index, 0].set_title(f"{label}\nOfficial HM3D projection")
        axes[index, 1].set_title("FALCON explored-map projection")
        axes[index, 2].set_title("Overlay and differences")
        for ax in axes[index]:
            ax.set_xlabel("FALCON x (m)")
            ax.set_ylabel("FALCON y (m)")
            ax.set_aspect("equal", adjustable="box")
        axes[index, 2].legend(handles=[
            Patch(facecolor="#222222", label="matched truth"),
            Patch(facecolor="#d50000", label="truth missing from map"),
            Patch(facecolor="#ff8a80", label="map-only difference"),
        ], loc="upper right", fontsize=8)
        floor_summaries.append({
            **floor,
            "slab_z_m": [float(z0), float(z1)],
            "truth_projection_cells": int(truth_mask.sum()),
            "matched_truth_cells": int(matched_truth.sum()),
            "missing_truth_cells": int(missing_truth.sum()),
            "map_only_cells": int(map_only.sum()),
        })

    platform_text = "none" if not platforms else ", ".join(
        f"z={item['falcon_floor_z_m']:.2f} m ({item['peak_fraction']:.1%})"
        for item in platforms
    )
    fig.suptitle(
        f"{scene_name} | {len(floors)} navigable level(s) | termination: {termination}\n"
        f"Platforms excluded from floor count: {platform_text} | difference tolerance: 0.30 m",
        fontsize=14,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return {"floors": floor_summaries, "excluded_platforms": platforms}


def set_equal_3d(ax, points: np.ndarray) -> None:
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    center = (lower + upper) * 0.5
    radius = max(float(np.max(upper - lower)) * 0.52, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def scatter_overlay(ax, truth: np.ndarray, occupied: np.ndarray) -> None:
    # Draw the map first and the official geometry last, otherwise the dense
    # voxel cloud hides the thin mesh boundaries in orthographic projections.
    ax.scatter(occupied[:, 0], occupied[:, 1], s=0.38, c="#2196f3", alpha=0.22,
               linewidths=0, rasterized=True, label="FALCON occupied", zorder=1)
    ax.scatter(truth[:, 0], truth[:, 1], s=0.42, c="#111111", alpha=0.58,
               linewidths=0, rasterized=True, label="official HM3D mesh", zorder=2)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--floor-output", type=Path)
    parser.add_argument("--max-truth-points", type=int, default=120_000)
    parser.add_argument("--max-map-points", type=int, default=120_000)
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    manifest = json.loads(
        (run_dir / "episode" / "manifest.json").read_text(encoding="utf-8")
    )
    scene = resolve_scene(str(args.scene or manifest["scene_path"]))
    scene_config = resolve_asset(manifest["scene_config"])
    occupied_all = load_ascii_pcd(run_dir / "bag_export" / "map_occupied.pcd")
    trajectory = load_trajectory(run_dir / "bag_export" / "trajectory.csv")
    truth_all = load_hm3d_vertices(scene, manifest)
    truth = deterministic_sample(truth_all, args.max_truth_points)
    occupied = deterministic_sample(occupied_all, args.max_map_points)
    combined = np.concatenate((truth, occupied), axis=0)

    output = args.output or run_dir / "comparison_3d" / "truth_vs_falcon_3d.png"
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(16, 12), constrained_layout=True)
    ax_truth = fig.add_subplot(2, 2, 1, projection="3d")
    ax_map = fig.add_subplot(2, 2, 2, projection="3d")
    ax_top = fig.add_subplot(2, 2, 3)
    ax_side = fig.add_subplot(2, 2, 4)

    ax_truth.scatter(
        truth[:, 0], truth[:, 1], truth[:, 2], s=0.38, c="#111111",
        alpha=0.52, linewidths=0, rasterized=True,
    )
    ax_truth.set_title("Official HM3D geometry (truth)")
    ax_map.scatter(
        occupied[:, 0], occupied[:, 1], occupied[:, 2], s=0.55,
        c="#1976d2", alpha=0.48, linewidths=0, rasterized=True,
    )
    if len(trajectory):
        ax_map.plot(
            trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
            color="#d32f2f", linewidth=1.4, label="executed trajectory",
        )
        ax_map.legend(loc="upper right", fontsize=8)
    ax_map.set_title("FALCON final occupied map + executed 3-D trajectory")
    for ax in (ax_truth, ax_map):
        ax.set_xlabel("FALCON x (m)")
        ax.set_ylabel("FALCON y (m)")
        ax.set_zlabel("FALCON z (m)")
        ax.view_init(elev=28, azim=-55)
        set_equal_3d(ax, combined)

    scatter_overlay(ax_top, truth, occupied)
    if len(trajectory):
        ax_top.plot(trajectory[:, 0], trajectory[:, 1], color="#d32f2f", linewidth=1.0)
    ax_top.set_title("Top overlay (x-y)")
    ax_top.set_xlabel("FALCON x (m)")
    ax_top.set_ylabel("FALCON y (m)")
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.legend(loc="best", fontsize=8)

    ax_side.scatter(occupied[:, 0], occupied[:, 2], s=0.38, c="#2196f3", alpha=0.22,
                    linewidths=0, rasterized=True, label="FALCON occupied", zorder=1)
    ax_side.scatter(truth[:, 0], truth[:, 2], s=0.42, c="#111111", alpha=0.58,
                    linewidths=0, rasterized=True, label="official HM3D mesh", zorder=2)
    if len(trajectory):
        ax_side.plot(trajectory[:, 0], trajectory[:, 2], color="#d32f2f", linewidth=1.0)
    ax_side.set_title("Vertical overlay (x-z)")
    ax_side.set_xlabel("FALCON x (m)")
    ax_side.set_ylabel("FALCON z (m)")
    ax_side.set_aspect("equal", adjustable="box")
    ax_side.legend(loc="best", fontsize=8)

    run_result_path = run_dir / "run_result.json"
    termination = "no completion record (interrupted or failed)"
    if run_result_path.is_file():
        termination = json.loads(run_result_path.read_text(encoding="utf-8")).get(
            "termination", "unknown"
        )
    fig.suptitle(
        f"{manifest['scene']} | official geometry vs FALCON map | termination: {termination}\n"
        "Visual geometry comparison only; vertex counts are not coverage percentages.",
        fontsize=14,
    )
    fig.savefig(output, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)

    floors, platforms = detect_hybrid_floors(
        scene, scene_config, manifest, occupied_all
    )
    if not floors:
        raise RuntimeError(f"No navigable floor levels detected for {scene}")
    floor_output = (
        args.floor_output
        or run_dir / "comparison_3d" / "truth_vs_falcon_floor_projections.png"
    ).resolve()
    floor_projection = plot_floor_projections(
        truth_all,
        occupied_all,
        floors,
        platforms,
        floor_output,
        manifest["scene"],
        termination,
    )

    summary = {
        "format": "pre_map_vln.stage1_3d_truth_comparison.v1",
        "scene": str(scene),
        "run_dir": str(run_dir),
        "termination": termination,
        "truth_mesh_vertices": int(len(truth_all)),
        "occupied_map_points": int(len(occupied_all)),
        "trajectory_points": int(len(trajectory)),
        "figure": str(output),
        "floor_projection_figure": str(floor_output),
        "detected_floor_count": len(floors),
        "floor_projection": floor_projection,
        "note": "Visual geometry comparison only; vertex counts are not coverage percentages.",
    }
    summary_path = output.with_suffix(".json")
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
