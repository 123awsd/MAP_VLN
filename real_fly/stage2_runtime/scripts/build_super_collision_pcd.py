#!/usr/bin/env python3
"""Build and cache an XYZ-only voxel map for SUPER's occupied prior."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_binary_xyz(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        header: list[str] = []
        while True:
            line = stream.readline()
            if not line:
                raise SystemExit("PCD header ended before DATA")
            header.append(line.decode("ascii").strip())
            if line.upper().startswith(b"DATA"):
                break

        values = {}
        for line in header:
            parts = line.split()
            if len(parts) >= 2:
                values[parts[0].upper()] = parts[1:]
        if values.get("DATA", [""])[0].lower() != "binary":
            raise SystemExit("SUPER collision-map builder currently requires binary PCD")

        fields = values["FIELDS"]
        sizes = [int(value) for value in values["SIZE"]]
        kinds = values["TYPE"]
        counts = [int(value) for value in values.get("COUNT", ["1"] * len(fields))]
        points = int(values["POINTS"][0])
        if not {"x", "y", "z"}.issubset(fields):
            raise SystemExit("PCD does not contain x/y/z fields")

        offsets = []
        formats = []
        offset = 0
        for name, size, kind, count in zip(fields, sizes, kinds, counts):
            offsets.append(offset)
            if kind != "F" or size != 4 or count != 1:
                raise SystemExit(f"unsupported PCD field encoding: {name} {kind}{size}x{count}")
            formats.append("<f4")
            offset += size * count
        cloud = np.fromfile(stream, dtype=np.dtype({
            "names": fields, "formats": formats,
            "offsets": offsets, "itemsize": offset,
        }), count=points)
        return np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(
            np.float32, copy=False)


def write_binary_xyz(path: Path, points: np.ndarray) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(np.ascontiguousarray(points, dtype="<f4").tobytes())


def load_approved_start(bundle_path: Path, source_hash: str) -> tuple[list[float], str]:
    bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    if bundle.get("format") != "pre_map_vln.real_execution_bundle.v1":
        raise SystemExit(f"unsupported execution bundle: {bundle.get('format')}")
    if bundle.get("map_sha256") != source_hash:
        raise SystemExit("execution bundle/map SHA256 mismatch")
    if not bundle.get("safety", {}).get("start_pose_explicitly_approved", False):
        raise SystemExit("execution bundle start pose is not explicitly approved")
    start = [float(value) for value in bundle.get("start_xyz_yaw", [])]
    if len(start) != 4 or not all(math.isfinite(value) for value in start):
        raise SystemExit("execution bundle has invalid start_xyz_yaw")
    return start, sha256(bundle_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel", type=float, default=0.10)
    parser.add_argument("--min-points-per-voxel", type=int, default=100)
    parser.add_argument("--bundle", type=Path,
                        help="approved execution bundle defining the launch hover pose")
    parser.add_argument("--launch-clearance-radius", type=float, default=0.60)
    parser.add_argument("--max-removed-voxels", type=int, default=100)
    args = parser.parse_args()
    if args.voxel < 0.1:
        raise SystemExit("--voxel must be at least the 0.10 m ROG resolution")
    if args.min_points_per_voxel < 1:
        raise SystemExit("--min-points-per-voxel must be positive")
    if not 0.35 <= args.launch_clearance_radius <= 0.60:
        raise SystemExit("--launch-clearance-radius must be in [0.35, 0.60] m")
    if not 1 <= args.max_removed_voxels <= 500:
        raise SystemExit("--max-removed-voxels must be in [1, 500]")

    source_hash = sha256(args.input)
    approved_start = None
    bundle_hash = None
    if args.bundle is not None:
        if not args.bundle.is_file():
            raise SystemExit(f"execution bundle not found: {args.bundle}")
        approved_start, bundle_hash = load_approved_start(args.bundle, source_hash)
    metadata_path = args.output.with_suffix(args.output.suffix + ".json")
    expected = {
        "format": "pre_map_vln.super_collision_pcd.v2",
        "source_sha256": source_hash,
        "voxel_m": args.voxel,
        "min_points_per_voxel": args.min_points_per_voxel,
        "launch_clearance_bundle_sha256": bundle_hash,
        "launch_clearance_center_xyz_m": approved_start[:3] if approved_start else None,
        "launch_clearance_radius_m": (
            args.launch_clearance_radius if approved_start else None
        ),
    }
    if args.output.is_file() and metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if all(metadata.get(key) == value for key, value in expected.items()):
            print(f"Reusing cached SUPER collision map: {args.output}")
            return

    xyz = read_binary_xyz(args.input)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    source_points = len(xyz)
    cells = np.floor(xyz / args.voxel).astype(np.int32)
    _, first, voxel_counts = np.unique(
        cells, axis=0, return_index=True, return_counts=True
    )
    keep = voxel_counts >= args.min_points_per_voxel
    reduced = xyz[np.sort(first[keep])]
    low_support_voxels_removed = int(np.count_nonzero(~keep))
    removed_voxels = 0
    if approved_start is not None:
        center = np.asarray(approved_start[:3], dtype=np.float32)
        distances = np.linalg.norm(reduced - center, axis=1)
        remove = distances < args.launch_clearance_radius
        removed_voxels = int(np.count_nonzero(remove))
        if removed_voxels > args.max_removed_voxels:
            raise SystemExit(
                f"launch clearance contains {removed_voxels} static voxels, exceeding "
                f"the safety limit {args.max_removed_voxels}; inspect the start area"
            )
        reduced = reduced[~remove]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(f".{args.output.name}.{os.getpid()}.tmp")
    write_binary_xyz(temporary, reduced)
    os.replace(temporary, args.output)
    metadata = {
        **expected,
        "source": str(args.input.resolve()),
        "source_points": source_points,
        "output_points": len(reduced),
        "low_support_voxels_removed": low_support_voxels_removed,
        "launch_clearance_removed_voxels": removed_voxels,
        "max_removed_voxels": args.max_removed_voxels,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"SUPER collision map: {source_points} -> {len(reduced)} points "
          f"({args.voxel:.2f} m voxel, minimum {args.min_points_per_voxel} points, "
          f"removed {low_support_voxels_removed} low-support voxels)")
    if approved_start is not None:
        print(f"Approved launch clearance: removed {removed_voxels} static voxels "
              f"inside {args.launch_clearance_radius:.2f} m of {approved_start[:3]}")


if __name__ == "__main__":
    main()
