#!/usr/bin/env python3
"""Build an immutable 3-D planning snapshot from FALCON's three state topics."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import tempfile
import uuid
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json  # noqa: E402
from stage2.planning_contract import (  # noqa: E402
    OCCUPANCY_SEMANTICS_HASH,
    OccupancyState,
    PlannerProfile,
)
from stage2.voxel_map_3d import VoxelMap3D  # noqa: E402


def load_ascii_pcd_xyz(path: Path) -> np.ndarray:
    header_lines = 0
    points_declared = None
    with path.open("r", encoding="ascii") as handle:
        for line in handle:
            header_lines += 1
            if line.startswith("POINTS "):
                points_declared = int(line.split()[1])
            if line.strip() == "DATA ascii":
                break
        else:
            raise ValueError(f"{path} is not an ASCII PCD")
    points = np.loadtxt(path, dtype=np.float32, skiprows=header_lines, usecols=(0, 1, 2))
    points = np.atleast_2d(points)
    if points_declared is not None and len(points) != points_declared:
        raise ValueError(f"{path} declares {points_declared} points but contains {len(points)}")
    return points


def configured_shape(config_path: Path, resolution: float) -> tuple[int, int, int]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    size = config["map_config"]["map_size"]
    lower = np.asarray([size[f"vbox_min_{axis}"] for axis in "xyz"], dtype=float)
    upper = np.asarray([size[f"vbox_max_{axis}"] for axis in "xyz"], dtype=float)
    shape = np.rint((upper - lower) / resolution).astype(int)
    if np.any(shape <= 0) or not np.allclose(lower + shape * resolution, upper, atol=1e-6):
        raise ValueError("vbox bounds are not aligned to the requested resolution")
    return tuple(int(value) for value in shape)


def points_to_flat(points: np.ndarray, origin: np.ndarray, resolution: float, shape_xyz) -> np.ndarray:
    indices = np.floor((points.astype(np.float64) - origin) / resolution + 1e-5).astype(np.int64)
    shape = np.asarray(shape_xyz, dtype=np.int64)
    if np.any(indices < 0) or np.any(indices >= shape):
        bad = points[np.any((indices < 0) | (indices >= shape), axis=1)][0]
        raise ValueError(f"PCD point lies outside configured vbox: {bad.tolist()}")
    return np.ravel_multi_index((indices[:, 2], indices[:, 1], indices[:, 0]), shape_xyz[::-1])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_export", type=Path)
    parser.add_argument("generated_config", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--planning-config", type=Path, default=ROOT / "config/uav_3d_planning.yaml")
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--map-epoch-uuid", default=None)
    parser.add_argument("--geometry-map-version", type=int, default=1)
    parser.add_argument("--semantic-map-version", type=int, default=1)
    args = parser.parse_args()

    profile = PlannerProfile.load(args.planning_config)
    nominal_shape_xyz = configured_shape(args.generated_config, args.resolution)
    point_sets = {}
    sources = {}
    counts = {}
    for name in ("unknown", "free", "occupied"):
        path = args.bag_export / f"map_{name}.pcd"
        point_sets[name] = load_ascii_pcd_xyz(path)
        counts[name] = int(len(point_sets[name]))
        sources[name] = str(path.resolve())
    # FALCON converts configured vbox bounds to integer indices with C++
    # floating-point floor().  The published voxel centers are authoritative;
    # deriving the boundary from those centers preserves the exact C++ grid.
    center_min = np.min(np.vstack([points.min(axis=0) for points in point_sets.values()]), axis=0)
    center_max = np.max(np.vstack([points.max(axis=0) for points in point_sets.values()]), axis=0)
    shape_xyz = tuple(
        int(value) for value in (np.rint((center_max - center_min) / args.resolution).astype(int) + 1)
    )
    if shape_xyz != nominal_shape_xyz:
        raise ValueError(
            f"published FALCON grid shape {shape_xyz} differs from configured vbox {nominal_shape_xyz}"
        )
    origin = center_min.astype(np.float64) - 0.5 * args.resolution
    shape_zyx = shape_xyz[::-1]
    raw = np.full(shape_zyx, int(OccupancyState.UNKNOWN), dtype=np.int8)
    assigned = np.zeros(shape_zyx, dtype=np.uint8)
    for name, state in (
        ("unknown", OccupancyState.UNKNOWN),
        ("free", OccupancyState.FREE),
        ("occupied", OccupancyState.OCCUPIED),
    ):
        points = point_sets.pop(name)
        flat = points_to_flat(points, origin, args.resolution, shape_xyz)
        if np.any(assigned.ravel()[flat]):
            raise ValueError(f"FALCON state point clouds overlap while assigning {name}")
        raw.ravel()[flat] = int(state)
        assigned.ravel()[flat] = 1
        del points, flat
    if not np.all(assigned == 1):
        raise ValueError(f"FALCON state PCDs leave {int(np.count_nonzero(assigned == 0))} voxels unassigned")

    esdf = distance_transform_edt(
        raw != int(OccupancyState.OCCUPIED), sampling=args.resolution
    ).astype(np.float32)
    provisional = {
        "format": "pre_map_vln.falcon_voxel_snapshot.v1",
        "frame_id": "falcon_world",
        "origin_xyz_m": origin.tolist(),
        "resolution_m": float(args.resolution),
        "shape_xyz": list(shape_xyz),
        "counts": counts,
        "sources": sources,
        "generated_config": str(args.generated_config.resolve()),
        "planning_config": str(args.planning_config.resolve()),
        "minimum_esdf_distance_m": profile.minimum_esdf_distance_m,
        "preferred_esdf_distance_m": profile.preferred_esdf_distance_m,
    }
    content = hashlib.sha256()
    content.update(raw.tobytes(order="C"))
    content.update(esdf.tobytes(order="C"))
    content.update(json.dumps(provisional, sort_keys=True).encode("utf-8"))
    identity = {
        "map_epoch_uuid": args.map_epoch_uuid or str(uuid.uuid4()),
        "geometry_map_version": int(args.geometry_map_version),
        "semantic_map_version": int(args.semantic_map_version),
        "snapshot_content_hash": content.hexdigest(),
        "occupancy_semantics_hash": OCCUPANCY_SEMANTICS_HASH,
        "planner_profile_hash": profile.profile_hash,
    }
    metadata = {**provisional, "identity": identity}

    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=args.output, suffix=".npz", delete=False) as handle:
        temporary = Path(handle.name)
    try:
        np.savez_compressed(temporary, raw_occupancy_zyx=raw, esdf_zyx_m=esdf)
        os.replace(temporary, args.output / "voxel_map.npz")
    finally:
        temporary.unlink(missing_ok=True)
    atomic_json(args.output / "metadata.json", metadata)
    loaded = VoxelMap3D.load(args.output, profile)
    if loaded.planning_state_signature() == "":
        raise AssertionError("planning state signature cannot be empty")
    print(json.dumps({
        "output": str(args.output), "shape_xyz": shape_xyz, "counts": counts,
        "identity": identity, "planning_state_signature": loaded.planning_state_signature(),
        "inflated_free_voxels": int(np.count_nonzero(loaded.inflated_free)),
    }, indent=2))


if __name__ == "__main__":
    main()
