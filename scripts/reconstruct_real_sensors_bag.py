#!/usr/bin/env python3
"""Recover, clean, classify, and RGB-colorize the readable part of sensors.bag.

This reader deliberately does not depend on ROS.  It understands the ROS1 bag
records and message layouts used by the real-robot recording.  That matters for
this particular bag because its final third and its trailing index are damaged;
normal rosbag iteration stops before exposing the otherwise valid prefix.

Required Python packages: lz4, numpy, scipy, pillow, matplotlib.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import lz4.frame
import matplotlib
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

matplotlib.use("Agg")
import matplotlib.pyplot as plt


POINT_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "nx", "ny", "nz", "intensity", "curvature"],
        "formats": ["<f4"] * 8,
        "offsets": [0, 4, 8, 16, 20, 24, 32, 36],
        "itemsize": 48,
    }
)
LABEL_NAMES = {0: "other", 1: "ground", 2: "wall", 3: "ceiling"}
LABEL_COLORS = np.array(
    [
        [150, 150, 150],
        [70, 190, 90],
        [70, 125, 235],
        [245, 190, 55],
    ],
    dtype=np.uint8,
)


def uint32(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def parse_fields(data: bytes) -> dict[str, bytes]:
    result: dict[str, bytes] = {}
    offset = 0
    while offset < len(data):
        length = uint32(data, offset)
        offset += 4
        field = data[offset : offset + length]
        offset += length
        key, value = field.split(b"=", 1)
        result[key.decode("ascii")] = value
    return result


def parse_string(data: bytes, offset: int) -> tuple[str, int]:
    length = uint32(data, offset)
    offset += 4
    return data[offset : offset + length].decode(errors="replace"), offset + length


def parse_header(data: bytes, offset: int = 0) -> tuple[tuple[int, int], str, int]:
    _, sec, nsec = struct.unpack_from("<III", data, offset)
    frame, offset = parse_string(data, offset + 12)
    return (sec, nsec), frame, offset


def inner_records(data: bytes) -> Iterator[tuple[dict[str, bytes], bytes]]:
    offset = 0
    while offset + 8 <= len(data):
        header_length = uint32(data, offset)
        offset += 4
        if not 5 <= header_length <= 100_000 or offset + header_length + 4 > len(data):
            return
        header = parse_fields(data[offset : offset + header_length])
        offset += header_length
        data_length = uint32(data, offset)
        offset += 4
        if offset + data_length > len(data):
            return
        yield header, data[offset : offset + data_length]
        offset += data_length


@dataclass(frozen=True)
class BagScan:
    messages: int
    chunks: int
    stop_offset: int


def iter_messages(path: Path, scan_result: list[BagScan] | None = None):
    connections: dict[int, str] = {}
    message_count = 0
    chunk_count = 0
    stop_offset = 0
    with path.open("rb") as stream:
        if stream.readline() != b"#ROSBAG V2.0\n":
            raise ValueError(f"Not a ROS1 bag: {path}")
        header_length = uint32(stream.read(4))
        stream.seek(header_length, 1)
        stream.seek(uint32(stream.read(4)), 1)

        while True:
            stop_offset = stream.tell()
            raw_length = stream.read(4)
            if len(raw_length) < 4:
                break
            header_length = uint32(raw_length)
            if not 5 <= header_length <= 100_000:
                break
            try:
                header = parse_fields(stream.read(header_length))
                data_length = uint32(stream.read(4))
                payload = stream.read(data_length)
            except (ValueError, struct.error):
                break
            if header.get("op", b"\0")[0] != 5:
                continue
            chunk_count += 1
            try:
                chunk = lz4.frame.decompress(payload)
            except RuntimeError:
                continue
            for inner_header, data in inner_records(chunk):
                operation = inner_header.get("op", b"\0")[0]
                connection = uint32(inner_header["conn"])
                if operation == 7:
                    connections[connection] = inner_header["topic"].decode(errors="replace")
                elif operation == 2 and connection in connections:
                    message_count += 1
                    yield connections[connection], inner_header, data
    if scan_result is not None:
        scan_result.append(BagScan(message_count, chunk_count, stop_offset))


def parse_point_cloud(data: bytes) -> np.ndarray:
    _, _, offset = parse_header(data)
    height, width = struct.unpack_from("<II", data, offset)
    offset += 8
    field_count = uint32(data, offset)
    offset += 4
    for _ in range(field_count):
        _, offset = parse_string(data, offset)
        offset += 9  # offset:uint32, datatype:uint8, count:uint32
    point_step = uint32(data, offset + 1)
    data_length = uint32(data, offset + 9)
    data_offset = offset + 13
    if point_step != POINT_DTYPE.itemsize:
        raise ValueError(f"Unexpected PointCloud2 point_step: {point_step}")
    expected = height * width * point_step
    if data_length < expected:
        raise ValueError("Truncated PointCloud2 message")
    points = np.frombuffer(
        data, dtype=POINT_DTYPE, count=height * width, offset=data_offset
    )
    return np.column_stack(
        (points["x"], points["y"], points["z"], points["intensity"])
    ).astype(np.float32, copy=False)


def parse_odometry(data: bytes) -> tuple[float, float, float]:
    _, _, offset = parse_header(data)
    _, offset = parse_string(data, offset)
    return struct.unpack_from("<ddd", data, offset)


def parse_camera_info(data: bytes) -> dict[str, object]:
    _, frame, offset = parse_header(data)
    height, width = struct.unpack_from("<II", data, offset)
    offset += 8
    distortion_model, offset = parse_string(data, offset)
    distortion_count = uint32(data, offset)
    offset += 4
    distortion = struct.unpack_from(f"<{distortion_count}d", data, offset)
    offset += 8 * distortion_count
    intrinsic = struct.unpack_from("<9d", data, offset)
    return {
        "frame": frame,
        "width": width,
        "height": height,
        "distortion_model": distortion_model,
        "distortion": list(distortion),
        "fx": intrinsic[0],
        "fy": intrinsic[4],
        "cx": intrinsic[2],
        "cy": intrinsic[5],
    }


def parse_marker_camera(data: bytes) -> tuple[tuple[int, int], np.ndarray, np.ndarray]:
    stamp, frame, offset = parse_header(data)
    if frame != "world":
        raise ValueError(f"Expected world-frame camera frustum, got {frame}")
    _, offset = parse_string(data, offset)
    _, marker_type, _ = struct.unpack_from("<iii", data, offset)
    offset += 12
    offset += 56  # pose
    offset += 24  # scale
    offset += 16  # color
    offset += 8  # lifetime
    offset += 1  # frame_locked
    point_count = uint32(data, offset)
    offset += 4
    points = np.frombuffer(data, dtype="<f8", count=point_count * 3, offset=offset).reshape(-1, 3)
    if marker_type != 5 or point_count < 8:  # visualization_msgs/Marker.LINE_LIST
        raise ValueError("Unexpected current-frustum marker layout")

    center = points[0].copy()
    top_left, top_right, bottom_right, bottom_left = points[[1, 3, 5, 7]]
    forward_raw = (top_left + top_right + bottom_right + bottom_left) / 4.0 - center
    right_raw = (top_right + bottom_right - top_left - bottom_left) / 2.0
    down_raw = (bottom_right + bottom_left - top_left - top_right) / 2.0

    forward = forward_raw / np.linalg.norm(forward_raw)
    right_raw -= forward * np.dot(right_raw, forward)
    right = right_raw / np.linalg.norm(right_raw)
    down_raw -= forward * np.dot(down_raw, forward)
    down_raw -= right * np.dot(down_raw, right)
    down = down_raw / np.linalg.norm(down_raw)
    if np.dot(np.cross(right, down), forward) < 0:
        down *= -1
    rotation_world_from_optical = np.column_stack((right, down, forward))
    return stamp, center, rotation_world_from_optical


def parse_rgb_image(data: bytes) -> tuple[tuple[int, int], np.ndarray]:
    stamp, _, offset = parse_header(data)
    height, width = struct.unpack_from("<II", data, offset)
    offset += 8
    encoding, offset = parse_string(data, offset)
    is_bigendian = data[offset]
    step = uint32(data, offset + 1)
    data_length = uint32(data, offset + 5)
    offset += 9
    if encoding != "rgb8" or is_bigendian or step != width * 3:
        raise ValueError(
            f"Unsupported image layout: {encoding=}, {is_bigendian=}, {step=}"
        )
    image = np.frombuffer(data, dtype=np.uint8, count=data_length, offset=offset)
    return stamp, image.reshape(height, step)[:, : width * 3].reshape(height, width, 3)


def write_ply(
    path: Path,
    xyz: np.ndarray,
    *,
    intensity: np.ndarray,
    normals: np.ndarray,
    rgb: np.ndarray,
    persistence: np.ndarray,
    labels: np.ndarray,
    color_count: np.ndarray | None = None,
) -> None:
    fields = [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("persistence", "<u2"),
        ("label", "u1"),
        ("red", "u1"),
        ("green", "u1"),
        ("blue", "u1"),
    ]
    if color_count is not None:
        fields.append(("color_count", "<u2"))
    vertices = np.empty(len(xyz), dtype=np.dtype(fields))
    for column, name in enumerate(("x", "y", "z")):
        vertices[name] = xyz[:, column]
    vertices["intensity"] = intensity
    for column, name in enumerate(("nx", "ny", "nz")):
        vertices[name] = normals[:, column]
    vertices["persistence"] = np.minimum(persistence, 65535).astype(np.uint16)
    vertices["label"] = labels
    vertices["red"], vertices["green"], vertices["blue"] = rgb.T
    if color_count is not None:
        vertices["color_count"] = np.minimum(color_count, 65535).astype(np.uint16)

    property_types = {
        "<f4": "float",
        "<u2": "ushort",
        "u1": "uchar",
        "|u1": "uchar",
    }
    header = [
        "ply",
        "format binary_little_endian 1.0",
        "comment recovered from readable prefix of sensors.bag",
        f"element vertex {len(vertices)}",
    ]
    for name in vertices.dtype.names:
        dtype = vertices.dtype.fields[name][0].str
        header.append(f"property {property_types[dtype]} {name}")
    header.append("end_header")
    with path.open("wb") as stream:
        stream.write(("\n".join(header) + "\n").encode())
        vertices.tofile(stream)


def estimate_normals(points: np.ndarray, neighbors: int = 16) -> tuple[np.ndarray, np.ndarray]:
    tree = cKDTree(points)
    normals = np.empty_like(points, dtype=np.float32)
    planarity = np.empty(len(points), dtype=np.float32)
    batch = 30_000
    for start in range(0, len(points), batch):
        stop = min(start + batch, len(points))
        _, indices = tree.query(points[start:stop], k=neighbors, workers=-1)
        local = points[indices].astype(np.float64)
        local -= local.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", local, local) / neighbors
        values, vectors = np.linalg.eigh(covariance)
        normals[start:stop] = vectors[:, :, 0]
        planarity[start:stop] = (values[:, 0] / np.maximum(values.sum(axis=1), 1e-12)).astype(
            np.float32
        )
    return normals, planarity


def classify_structure(
    xyz: np.ndarray, normals: np.ndarray, planarity: np.ndarray, trajectory: np.ndarray
) -> np.ndarray:
    labels = np.zeros(len(xyz), dtype=np.uint8)
    planar = planarity < 0.075
    abs_normal_z = np.abs(normals[:, 2])
    if len(trajectory):
        trajectory_tree = cKDTree(trajectory[:, :2])
        _, nearest = trajectory_tree.query(xyz[:, :2], k=1, workers=-1)
        relative_height = xyz[:, 2] - trajectory[nearest, 2]
    else:
        relative_height = xyz[:, 2] - np.median(xyz[:, 2])
    labels[planar & (abs_normal_z < 0.38)] = 2
    labels[planar & (abs_normal_z > 0.78) & (relative_height <= 0.35)] = 1
    labels[planar & (abs_normal_z > 0.78) & (relative_height >= 1.55)] = 3
    return labels


def select_keyframes(
    cameras: list[tuple[tuple[int, int], np.ndarray, np.ndarray]]
) -> list[tuple[tuple[int, int], np.ndarray, np.ndarray]]:
    selected = []
    for camera in cameras:
        stamp, center, rotation = camera
        if not selected:
            selected.append(camera)
            continue
        last_stamp, last_center, last_rotation = selected[-1]
        distance = np.linalg.norm(center - last_center)
        angle = math.acos(
            float(np.clip(np.dot(rotation[:, 2], last_rotation[:, 2]), -1.0, 1.0))
        )
        elapsed = (stamp[0] - last_stamp[0]) + (stamp[1] - last_stamp[1]) * 1e-9
        if distance >= 0.12 or angle >= math.radians(6.0) or elapsed >= 0.5:
            selected.append(camera)
    return selected


def project_color(
    xyz: np.ndarray,
    image: np.ndarray,
    center: np.ndarray,
    rotation_world_from_optical: np.ndarray,
    camera: dict[str, object],
    color_sum: np.ndarray,
    color_count: np.ndarray,
) -> int:
    camera_points = (xyz - center) @ rotation_world_from_optical
    depth = camera_points[:, 2]
    valid = (depth > 0.15) & (depth < 20.0)
    indices = np.flatnonzero(valid)
    if not len(indices):
        return 0
    local = camera_points[indices]
    u = np.rint(float(camera["fx"]) * local[:, 0] / local[:, 2] + float(camera["cx"])).astype(
        np.int32
    )
    v = np.rint(float(camera["fy"]) * local[:, 1] / local[:, 2] + float(camera["cy"])).astype(
        np.int32
    )
    width, height = int(camera["width"]), int(camera["height"])
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    indices, u, v, local_depth = indices[inside], u[inside], v[inside], local[inside, 2]
    if not len(indices):
        return 0

    pixels = v * width + u
    depth_buffer = np.full(width * height, np.inf, dtype=np.float32)
    np.minimum.at(depth_buffer, pixels, local_depth)
    visible = local_depth <= depth_buffer[pixels] + 0.10 + 0.005 * local_depth
    indices, u, v = indices[visible], u[visible], v[visible]
    colors = image[v, u].astype(np.uint32)
    color_sum[indices] += colors
    color_count[indices] += 1
    return len(indices)


def save_structure_preview(path: Path, xyz: np.ndarray, labels: np.ndarray) -> None:
    stride = max(1, len(xyz) // 300_000)
    points, colors = xyz[::stride], LABEL_COLORS[labels[::stride]] / 255.0
    figure = plt.figure(figsize=(14, 6), dpi=160)
    top = figure.add_subplot(121)
    top.scatter(points[:, 0], points[:, 1], c=colors, s=0.3)
    top.set_aspect("equal")
    top.set_title("Clean structural map")
    top.set_xlabel("X [m]")
    top.set_ylabel("Y [m]")
    view = figure.add_subplot(122, projection="3d")
    view.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=0.2)
    view.view_init(28, -60)
    view.set_title("green=ground, blue=wall, yellow=ceiling")
    figure.tight_layout()
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)


def save_rgb_preview(path: Path, xyz: np.ndarray, rgb: np.ndarray, colored: np.ndarray) -> None:
    points, colors = xyz[colored], rgb[colored]
    stride = max(1, len(points) // 300_000)
    points, colors = points[::stride], colors[::stride] / 255.0
    figure = plt.figure(figsize=(14, 6), dpi=160)
    top = figure.add_subplot(121)
    top.scatter(points[:, 0], points[:, 1], c=colors, s=0.35)
    top.set_aspect("equal")
    top.set_title("RGB-colored map (top-down)")
    top.set_xlabel("X [m]")
    top.set_ylabel("Y [m]")
    view = figure.add_subplot(122, projection="3d")
    view.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=0.2)
    view.view_init(22, -55)
    view.set_title("RGB observations only")
    figure.tight_layout()
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--min-persistence", type=int, default=3)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    clouds: list[np.ndarray] = []
    trajectory: list[tuple[float, float, float]] = []
    cameras: list[tuple[tuple[int, int], np.ndarray, np.ndarray]] = []
    camera_info = None
    scan_result: list[BagScan] = []
    for topic, _, data in iter_messages(args.bag, scan_result):
        if topic == "/cloud_registered":
            clouds.append(parse_point_cloud(data))
        elif topic == "/Odometry":
            trajectory.append(parse_odometry(data))
        elif topic == "/camera/color/camera_info" and camera_info is None:
            camera_info = parse_camera_info(data)
        elif topic == "/rgb_coverage/current_frustum":
            cameras.append(parse_marker_camera(data))
    if not clouds or camera_info is None or not cameras:
        raise RuntimeError("Bag lacks required cloud, camera-info, or camera-frustum messages")

    raw = np.concatenate(clouds)
    del clouds
    valid = np.isfinite(raw).all(axis=1) & (np.linalg.norm(raw[:, :3], axis=1) < 100.0)
    raw = raw[valid]
    raw_count = len(raw)
    voxel_keys = np.floor(raw[:, :3] / args.voxel_size).astype(np.int32)
    _, inverse, persistence = np.unique(
        voxel_keys, axis=0, return_inverse=True, return_counts=True
    )
    del voxel_keys
    voxel_count = len(persistence)
    averaged = np.empty((voxel_count, 4), dtype=np.float32)
    for column in range(4):
        averaged[:, column] = np.bincount(inverse, weights=raw[:, column]) / persistence
    del inverse, raw

    stable = persistence >= args.min_persistence
    averaged, persistence = averaged[stable], persistence[stable]
    neighbor_tree = cKDTree(averaged[:, :3])
    neighbor_distance, _ = neighbor_tree.query(averaged[:, :3], k=4, workers=-1)
    locally_supported = neighbor_distance[:, 3] <= max(0.18, args.voxel_size * 3.6)
    averaged, persistence = averaged[locally_supported], persistence[locally_supported]
    xyz, intensity = averaged[:, :3], averaged[:, 3]

    normals, planarity = estimate_normals(xyz)
    trajectory_array = np.asarray(trajectory, dtype=np.float32)
    labels = classify_structure(xyz, normals, planarity, trajectory_array)
    label_rgb = LABEL_COLORS[labels]

    clean_ply = args.output_dir / "map_building_clean_5cm.ply"
    write_ply(
        clean_ply,
        xyz,
        intensity=intensity,
        normals=normals,
        rgb=label_rgb,
        persistence=persistence,
        labels=labels,
    )
    class_outputs: dict[str, str] = {}
    for label, name in ((1, "ground"), (2, "walls"), (3, "ceiling")):
        selected = labels == label
        class_path = args.output_dir / f"map_{name}_5cm.ply"
        write_ply(
            class_path,
            xyz[selected],
            intensity=intensity[selected],
            normals=normals[selected],
            rgb=label_rgb[selected],
            persistence=persistence[selected],
            labels=labels[selected],
        )
        class_outputs[name] = str(class_path)
    save_structure_preview(
        args.output_dir / "map_building_clean_5cm_preview.png", xyz, labels
    )

    keyframes = select_keyframes(cameras)
    camera_by_stamp = {stamp: (center, rotation) for stamp, center, rotation in keyframes}
    color_sum = np.zeros((len(xyz), 3), dtype=np.uint32)
    color_count = np.zeros(len(xyz), dtype=np.uint16)
    projected_observations = 0
    matched_keyframes = 0
    for topic, _, data in iter_messages(args.bag):
        if topic != "/camera/color/image_raw":
            continue
        stamp, image = parse_rgb_image(data)
        pose = camera_by_stamp.get(stamp)
        if pose is None:
            continue
        projected_observations += project_color(
            xyz, image, pose[0], pose[1], camera_info, color_sum, color_count
        )
        matched_keyframes += 1

    colored = color_count > 0
    rgb = np.full((len(xyz), 3), 135, dtype=np.uint8)
    rgb[colored] = np.clip(
        color_sum[colored] / color_count[colored, None], 0, 255
    ).astype(np.uint8)
    rgb_ply = args.output_dir / "map_rgb_colored_5cm.ply"
    write_ply(
        rgb_ply,
        xyz,
        intensity=intensity,
        normals=normals,
        rgb=rgb,
        persistence=persistence,
        labels=labels,
        color_count=color_count,
    )
    observed_ply = args.output_dir / "map_rgb_colored_observed_5cm.ply"
    write_ply(
        observed_ply,
        xyz[colored],
        intensity=intensity[colored],
        normals=normals[colored],
        rgb=rgb[colored],
        persistence=persistence[colored],
        labels=labels[colored],
        color_count=color_count[colored],
    )
    trajectory_tree = cKDTree(trajectory_array[:, :2])
    _, nearest_trajectory = trajectory_tree.query(xyz[:, :2], k=1, workers=-1)
    relative_height = xyz[:, 2] - trajectory_array[nearest_trajectory, 2]
    roof_reference = (labels == 3) & (relative_height > 2.0)
    non_ceiling = labels != 3
    # The lidar/body origin stays about 1.5 m above the local floor.  Keeping
    # 2.4 m above that origin preserves roughly 3.9 m of wall height while
    # removing the vertical wisps and disconnected returns around tall roofs.
    roof_noise = relative_height > 2.4
    if np.any(roof_reference):
        roof_indices = np.flatnonzero(roof_reference)
        roof_tree = cKDTree(xyz[roof_indices, :2])
        horizontal_distance, nearest_roof = roof_tree.query(
            xyz[:, :2], k=1, workers=-1
        )
        local_roof_height = xyz[roof_indices[nearest_roof], 2]
        roof_noise |= (horizontal_distance < 0.5) & (xyz[:, 2] >= local_roof_height)
    roofless_clean = non_ceiling & ~roof_noise
    # Low ceilings can fall just below the semantic ceiling threshold when the
    # robot changes elevation.  Detect horizontal surfaces at least 0.75 m
    # above the lidar/body origin, then grow the removal by 18 cm to include
    # their noisy borders and the vertical wisps attached to those borders.
    low_roof_seed = (
        roofless_clean
        & (np.abs(normals[:, 2]) > 0.62)
        & (relative_height > 0.75)
        & (labels != 1)
    )
    if np.any(low_roof_seed):
        low_roof_tree = cKDTree(xyz[low_roof_seed])
        low_roof_distance, _ = low_roof_tree.query(xyz, k=1, workers=-1)
        roofless_clean &= low_roof_distance >= 0.18
    roofless_ply = args.output_dir / "map_rgb_roofless_clean_5cm.ply"
    write_ply(
        roofless_ply,
        xyz[roofless_clean],
        intensity=intensity[roofless_clean],
        normals=normals[roofless_clean],
        rgb=rgb[roofless_clean],
        persistence=persistence[roofless_clean],
        labels=labels[roofless_clean],
        color_count=color_count[roofless_clean],
    )
    save_rgb_preview(
        args.output_dir / "map_rgb_colored_5cm_preview.png", xyz, rgb, colored
    )

    counts = {LABEL_NAMES[label]: int(np.count_nonzero(labels == label)) for label in LABEL_NAMES}
    report = {
        "source_bag": str(args.bag),
        "readable_scan": scan_result[0].__dict__,
        "cloud_frames": len(trajectory),
        "raw_valid_points": raw_count,
        "voxel_size_m": args.voxel_size,
        "min_persistence": args.min_persistence,
        "clean_points": len(xyz),
        "class_counts": counts,
        "camera_frustums": len(cameras),
        "selected_keyframes": len(keyframes),
        "matched_keyframes": matched_keyframes,
        "projected_color_observations": projected_observations,
        "rgb_colored_points": int(colored.sum()),
        "rgb_coverage_fraction": float(colored.mean()),
        "roof_and_overhead_points_removed": int((~roofless_clean).sum()),
        "roofless_clean_points": int(roofless_clean.sum()),
        "bounds_min": xyz.min(axis=0).tolist(),
        "bounds_max": xyz.max(axis=0).tolist(),
        "outputs": {
            "clean_ply": str(clean_ply),
            **class_outputs,
            "rgb_ply": str(rgb_ply),
            "rgb_observed_ply": str(observed_ply),
            "rgb_roofless_clean_ply": str(roofless_ply),
        },
    }
    report_path = args.output_dir / "reconstruction_report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
