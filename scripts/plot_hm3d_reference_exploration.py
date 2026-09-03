#!/usr/bin/env python3
"""Render exploration coverage on uniformly sampled official HM3D geometry."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import magnum.trade as trade
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from plot_exploded_exploration_progress import depth_endpoints, exploded, floor_indices
from plot_partial_exploration_pointcloud import voxel_first


FALCON_SENSOR_ORIGIN = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)


def sample_hm3d_surface(scene: Path, manifest: dict, density: float, seed: int) -> np.ndarray:
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    if importer is None:
        raise RuntimeError("Magnum AnySceneImporter is unavailable")
    importer.open_file(str(scene))
    rng = np.random.default_rng(seed)
    chunks = []
    for mesh_index in range(importer.mesh_count):
        mesh = importer.mesh(mesh_index)
        if not mesh.has_attribute(trade.MeshAttribute.POSITION):
            continue
        vertices = np.asarray(mesh.attribute(trade.MeshAttribute.POSITION), dtype=np.float64)
        indices = np.asarray(mesh.indices).reshape(-1) if mesh.is_indexed else np.arange(len(vertices))
        triangles = vertices[indices[: len(indices) // 3 * 3]].reshape(-1, 3, 3)
        cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        area = 0.5 * np.linalg.norm(cross, axis=1)
        expected = area * density
        counts = np.floor(expected).astype(np.int32)
        counts += rng.random(len(counts)) < (expected - counts)
        selected = np.repeat(np.arange(len(triangles)), counts)
        if not len(selected):
            continue
        first = rng.random(len(selected))
        second = rng.random(len(selected))
        root = np.sqrt(first)
        weights = np.column_stack((1.0 - root, root * (1.0 - second), root * second))
        chunks.append(np.einsum("ni,nij->nj", weights, triangles[selected]))
    basis = np.concatenate(chunks)
    habitat = np.column_stack((basis[:, 0], basis[:, 2], -basis[:, 1]))
    initial_sensor = np.asarray(manifest["initial_sensor_habitat_xyz"], dtype=np.float64)
    transform = np.asarray(manifest["main_floor_mapping"]["transform"], dtype=np.float64)
    return ((habitat - initial_sensor) @ transform.T + FALCON_SENSOR_ORIGIN).astype(np.float32)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("scene", type=Path)
    parser.add_argument("floors_json", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--cutoff-sequence", type=int, default=640)
    parser.add_argument("--surface-density", type=float, default=260.0,
                        help="uniform HM3D samples per square meter")
    parser.add_argument("--depth-stride", type=int, default=4)
    parser.add_argument("--depth-voxel", type=float, default=0.045)
    parser.add_argument("--match-radius", type=float, default=0.085)
    parser.add_argument("--maximum-depth", type=float, default=6.0)
    parser.add_argument("--top-clearance", type=float, default=2.30)
    parser.add_argument("--floor-spacing", type=float, default=3.7)
    parser.add_argument("--seed", type=int, default=337)
    args = parser.parse_args()

    manifest = json.loads((args.run_dir / "episode/manifest.json").read_text())
    floor_data = json.loads(args.floors_json.read_text())
    floor_z = np.asarray([item["floor_z_m"] for item in floor_data], dtype=np.float32)
    frames = [
        path for path in sorted((args.run_dir / "episode").glob("frame_*.npz"))
        if int(path.stem.split("_")[-1]) <= args.cutoff_sequence
    ]
    if not frames:
        raise FileNotFoundError("no archived frame before cutoff")

    reference = sample_hm3d_surface(args.scene, manifest, args.surface_density, args.seed)
    reference_floor = floor_indices(reference, floor_z, args.top_clearance)
    keep = reference_floor >= 0
    reference, reference_floor = reference[keep], reference_floor[keep]

    observed_parts, trajectory = [], []
    for path in frames:
        points, position = depth_endpoints(path, args.depth_stride, args.maximum_depth)
        observed_parts.append(points)
        trajectory.append(position)
    measured, _ = voxel_first(np.concatenate(observed_parts), args.depth_voxel)
    distance, _ = cKDTree(measured).query(reference, k=1, workers=-1)
    observed = distance <= args.match_radius

    shown_reference = exploded(reference, reference_floor, floor_z, args.floor_spacing)
    trajectory = np.asarray(trajectory)
    trajectory_floor = np.argmin(np.abs(trajectory[:, None, 2] - (floor_z[None, :] + 1.2)), axis=1)
    shown_trajectory = exploded(trajectory, trajectory_floor, floor_z, args.floor_spacing)

    fig = plt.figure(figsize=(8.6, 7.2), dpi=220, facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(*shown_reference[~observed].T, s=1.05, marker="s", c="#aeb4ba",
               alpha=.58, linewidths=0, depthshade=False, rasterized=True)
    ax.scatter(*shown_reference[observed].T, s=1.18, marker="s", c="#079bbd",
               alpha=1.0, linewidths=0, depthshade=False, rasterized=True)

    for floor in range(len(floor_z)):
        member = trajectory_floor == floor
        starts = np.flatnonzero(member & np.r_[True, ~member[:-1]])
        ends = np.flatnonzero(member & np.r_[~member[1:], True]) + 1
        for start, end in zip(starts, ends):
            if end - start >= 2:
                ax.plot(*shown_trajectory[start:end].T, color="#ee7b2d", linewidth=.85,
                        alpha=.78, zorder=9)
    ax.scatter(*shown_trajectory[-1], s=34, c="#d9363e", marker="D",
               edgecolors="white", linewidths=.65, depthshade=False, zorder=11)

    low = np.percentile(shown_reference, .15, axis=0)
    high = np.percentile(shown_reference, 99.85, axis=0)
    ax.set_xlim(low[0] - .25, high[0] + .25)
    ax.set_ylim(low[1] - .25, high[1] + .25)
    ax.set_zlim(-.15, args.floor_spacing * (len(floor_z) - 1) + args.top_clearance + .15)
    ax.set_box_aspect((high[0] - low[0], high[1] - low[1], 10.0))
    ax.view_init(elev=25, azim=-63)
    ax.set_axis_off()
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=300, bbox_inches="tight", pad_inches=0, facecolor="white")
    pdf = args.output.with_suffix(".pdf")
    fig.savefig(pdf, bbox_inches="tight", pad_inches=0, facecolor="white")
    plt.close(fig)
    report = {
        "format": "pre_map_vln.hm3d_reference_exploration_figure.v1",
        "reference_geometry": str(args.scene.resolve()),
        "reference_surface_samples": int(len(reference)),
        "observed_reference_samples": int(observed.sum()),
        "observed_fraction_of_shown_reference": float(observed.mean()),
        "cutoff_sequence": args.cutoff_sequence,
        "archived_frames_used": len(frames),
        "measured_rgbd_surface_points": int(len(measured)),
        "reference_mesh_used_for_visualization_only": True,
        "png": str(args.output.resolve()), "pdf": str(pdf.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
