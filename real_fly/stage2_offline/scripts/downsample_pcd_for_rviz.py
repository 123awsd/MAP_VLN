#!/usr/bin/env python3
"""Create a lightweight XYZ-only PCD copy for RViz visualization."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np


def read_binary_xyz(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise SystemExit("PCD header ended before DATA")
            header.append(line.decode("ascii").rstrip("\n"))
            if line.startswith(b"DATA"):
                break
        values = {}
        for line in header:
            parts = line.split()
            if len(parts) >= 2:
                values[parts[0].upper()] = parts[1:]
        if values.get("DATA", [""])[0].lower() != "binary":
            raise SystemExit("only binary PCD is supported")
        fields = values["FIELDS"]
        sizes = [int(value) for value in values["SIZE"]]
        types = values["TYPE"]
        counts = [int(value) for value in values.get("COUNT", ["1"] * len(fields))]
        points = int(values["POINTS"][0])
        offsets = {}
        offset = 0
        dtype_fields = []
        for name, size, kind, count in zip(fields, sizes, types, counts):
            offsets[name] = offset
            dtype_fields.append((name, "<f4" if kind == "F" and size == 4 else None, count))
            if kind == "F" and size == 4:
                offset += 4 * count
            else:
                raise SystemExit(f"unsupported PCD field encoding: {name} {kind}{size}")
        raw = np.fromfile(stream, dtype=np.dtype({
            "names": fields,
            "formats": ["<f4"] * len(fields),
            "offsets": [offsets[name] for name in fields],
            "itemsize": offset,
        }), count=points)
        return np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32, copy=False)


def write_binary_xyz(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(np.ascontiguousarray(points, dtype="<f4").tobytes())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel", type=float, default=0.05,
                        help="visualization voxel size in metres (default: 0.05)")
    args = parser.parse_args()
    if args.voxel <= 0:
        raise SystemExit("--voxel must be positive")
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite: {args.output}")
    xyz = read_binary_xyz(args.input)
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]
    cells = np.floor(xyz / args.voxel).astype(np.int64)
    _, first = np.unique(cells, axis=0, return_index=True)
    reduced = xyz[np.sort(first)]
    write_binary_xyz(args.output, reduced)
    print(f"input_points={len(xyz)} output_points={len(reduced)} voxel_m={args.voxel}")


if __name__ == "__main__":
    main()
