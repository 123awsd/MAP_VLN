#!/usr/bin/env python3
"""Export synchronized real RGB-D/pose data and estimate camera extrinsics.

The output follows the subset of ScanNet's on-disk contract consumed by
Boxer's ScanNetLoader.  The input bag is opened read-only and output creation
is refused when the target already exists.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
import rosbag
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


TOPICS = {
    "odom": "/Odometry",
    "rgb": "/camera/color/image_raw",
    "depth": "/camera/aligned_depth_to_color/image_raw",
    "info": "/camera/color/camera_info",
    "tf_static": "/tf_static",
}


def stamp(msg, bag_time) -> float:
    value = getattr(getattr(getattr(msg, "header", None), "stamp", None), "to_sec", lambda: 0.0)()
    return float(value if value > 0 else bag_time.to_sec())


def pose_matrix(msg) -> np.ndarray:
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def transform_matrix(transform) -> np.ndarray:
    p, q = transform.translation, transform.rotation
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def image_array(msg) -> np.ndarray:
    dtype = np.dtype(msg.is_bigendian and ">u2" or "<u2") if msg.encoding in ("16UC1", "mono16") else np.uint8
    channels = 1 if msg.encoding in ("16UC1", "mono16", "mono8") else 3
    row_items = msg.step // np.dtype(dtype).itemsize
    raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_items)
    raw = raw[:, : msg.width * channels]
    return raw.reshape(msg.height, msg.width) if channels == 1 else raw.reshape(msg.height, msg.width, channels)


def cloud_xyz(msg) -> np.ndarray:
    offsets = {field.name: field.offset for field in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.width * msg.height, msg.point_step)
    columns = [raw[:, offsets[name] : offsets[name] + 4].copy().view("<f4").reshape(-1) for name in ("x", "y", "z")]
    return np.column_stack(columns).astype(np.float64)


def nearest_index(values: list[float], target: float) -> int:
    idx = bisect.bisect_left(values, target)
    choices = [max(0, idx - 1), min(len(values) - 1, idx)]
    return min(choices, key=lambda item: abs(values[item] - target))


def load_pcd_xyz(path: Path) -> np.ndarray:
    fields, sizes, types, counts, point_count = None, None, None, None, None
    with path.open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("incomplete PCD header")
            text = line.decode("ascii", errors="strict").strip()
            if text.startswith("FIELDS "): fields = text.split()[1:]
            elif text.startswith("SIZE "): sizes = [int(x) for x in text.split()[1:]]
            elif text.startswith("TYPE "): types = text.split()[1:]
            elif text.startswith("COUNT "): counts = [int(x) for x in text.split()[1:]]
            elif text.startswith("POINTS "): point_count = int(text.split()[1])
            elif text == "DATA binary":
                offset = handle.tell()
                break
        if not all(value is not None for value in (fields, sizes, types, counts, point_count)):
            raise ValueError("PCD header lacks fields")
        formats = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4", ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4"}
        dtype = []
        for name, size, kind, count in zip(fields, sizes, types, counts):
            code = formats[(kind, size)]
            dtype.append((name, code) if count == 1 else (name, code, (count,)))
        handle.seek(offset)
        data = np.fromfile(handle, dtype=np.dtype(dtype), count=point_count)
    return np.column_stack([data["x"], data["y"], data["z"]]).astype(np.float64)


def voxel_downsample(points: np.ndarray, resolution: float) -> np.ndarray:
    finite = points[np.isfinite(points).all(axis=1)]
    keys = np.floor(finite / resolution).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return finite[np.sort(indices)]


def depth_points(depth_mm: np.ndarray, K: np.ndarray, stride: int = 20) -> np.ndarray:
    vv, uu = np.mgrid[0:depth_mm.shape[0]:stride, 0:depth_mm.shape[1]:stride]
    z = depth_mm[vv, uu].astype(np.float64) / 1000.0
    good = (z > 0.45) & (z < 10.0)
    z, u, v = z[good], uu[good], vv[good]
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    return np.column_stack([x, y, z])


def parameter_matrix(values: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rotation.from_rotvec(values[3:]).as_matrix()
    out[:3, 3] = values[:3]
    return out


def atomic_json(path: Path, value) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(str(temporary), str(path))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("map_pcd", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--frame-period", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=180)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite: {args.output}")
    if not args.bag.is_file() or not args.map_pcd.is_file():
        raise SystemExit("bag or PCD input is missing")

    odom_times, odom_poses, rgb_times, depth_times = [], [], [], []
    K, camera_link_to_optical = None, None
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, bt in bag.read_messages(topics=list(TOPICS.values())):
            current = stamp(msg, bt)
            if topic == TOPICS["odom"]:
                odom_times.append(current); odom_poses.append(pose_matrix(msg))
            elif topic == TOPICS["rgb"]: rgb_times.append(current)
            elif topic == TOPICS["depth"]: depth_times.append(current)
            elif topic == TOPICS["info"] and K is None: K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
            elif topic == TOPICS["tf_static"]:
                for item in msg.transforms:
                    if item.header.frame_id == "camera_link" and item.child_frame_id == "camera_aligned_depth_to_color_frame":
                        first = transform_matrix(item.transform)
                    elif item.header.frame_id == "camera_aligned_depth_to_color_frame" and item.child_frame_id == "camera_color_optical_frame":
                        second = transform_matrix(item.transform)
                        if "first" in locals(): camera_link_to_optical = first @ second
    if not odom_times or not rgb_times or not depth_times or K is None:
        raise SystemExit("required odometry/RGB/depth/CameraInfo stream is missing")
    if camera_link_to_optical is None:
        camera_link_to_optical = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)

    valid_depth = [t for t in depth_times if odom_times[0] <= t <= odom_times[-1]]
    selected, last = [], -math.inf
    for value in valid_depth:
        if value - last >= args.frame_period:
            selected.append(value); last = value
        if len(selected) >= args.max_frames: break
    selected_rgb = [rgb_times[nearest_index(rgb_times, value)] for value in selected]
    depth_lookup = {round(value, 6): idx for idx, value in enumerate(selected)}
    rgb_lookup = {round(value, 6): idx for idx, value in enumerate(selected_rgb)}

    root = args.output
    color_dir, depth_dir, pose_dir = root / "frames/color", root / "frames/depth", root / "frames/pose"
    intrinsic_dir = root / "frames/intrinsic"
    for path in (color_dir, depth_dir, pose_dir, intrinsic_dir, root / "calibration", root / "reports"):
        path.mkdir(parents=True, exist_ok=False)
    depth_arrays = [None] * len(selected)
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, bt in bag.read_messages(topics=[TOPICS["rgb"], TOPICS["depth"]]):
            key = round(stamp(msg, bt), 6)
            if topic == TOPICS["rgb"] and key in rgb_lookup:
                idx = rgb_lookup[key]; image = image_array(msg)
                if msg.encoding == "rgb8": image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(color_dir / f"{idx:06d}.jpg"), image, [cv2.IMWRITE_JPEG_QUALITY, 94])
            elif topic == TOPICS["depth"] and key in depth_lookup:
                idx = depth_lookup[key]; image = image_array(msg).astype(np.uint16)
                depth_arrays[idx] = image
                cv2.imwrite(str(depth_dir / f"{idx:06d}.png"), image)

    calibration_samples = []
    for idx in range(0, len(selected), max(1, len(selected) // 30)):
        if depth_arrays[idx] is None: continue
        points = depth_points(depth_arrays[idx], K, stride=24)
        if len(points) < 50: continue
        body_pose = odom_poses[nearest_index(odom_times, selected[idx])]
        calibration_samples.append([selected[idx], body_pose, points, None])
    if len(calibration_samples) < 8:
        raise SystemExit("too few valid RGB-D/odometry calibration samples")

    # Pair every calibration depth frame with the closest registered LiDAR
    # scan.  Using the concurrent scan avoids false matches against repeated
    # walls elsewhere in the accumulated room map.
    sample_times = [item[0] for item in calibration_samples]
    best_dt = [math.inf] * len(calibration_samples)
    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, msg, bt in bag.read_messages(topics=["/cloud_registered"]):
            current = stamp(msg, bt)
            idx = nearest_index(sample_times, current)
            delta = abs(sample_times[idx] - current)
            if delta < best_dt[idx] and delta <= 0.10:
                points = cloud_xyz(msg)
                points = voxel_downsample(points[np.isfinite(points).all(axis=1)], 0.08)
                calibration_samples[idx][3] = cKDTree(points)
                best_dt[idx] = delta
    calibration_samples = [item for item in calibration_samples if item[3] is not None]
    if len(calibration_samples) < 8:
        raise SystemExit("too few time-paired depth/LiDAR scans for targetless calibration")

    def residual(values, samples):
        body_to_optical = parameter_matrix(values) @ camera_link_to_optical
        result = []
        for _, body_pose, points, tree in samples:
            world = (body_pose @ body_to_optical @ np.column_stack([points, np.ones(len(points))]).T).T[:, :3]
            distances = tree.query(world, k=1, workers=-1)[0]
            result.append(np.minimum(distances, 0.75))
        return np.concatenate(result)

    train = calibration_samples[::2]
    validation = calibration_samples[1::2] or train
    initial = np.zeros(6, dtype=np.float64)
    before = residual(initial, validation)
    fit = least_squares(residual, initial, args=(train,), loss="soft_l1", f_scale=0.12,
                        bounds=([-0.25] * 3 + [-0.40] * 3, [0.25] * 3 + [0.40] * 3), max_nfev=80, verbose=1)
    after = residual(fit.x, validation)
    body_to_link = parameter_matrix(fit.x)
    body_to_optical = body_to_link @ camera_link_to_optical

    intrinsic = np.eye(4); intrinsic[:3, :3] = K
    np.savetxt(str(intrinsic_dir / "intrinsic_color.txt"), intrinsic, fmt="%.12g")
    np.savetxt(str(intrinsic_dir / "intrinsic_depth.txt"), intrinsic, fmt="%.12g")
    for idx, current in enumerate(selected):
        body_pose = odom_poses[nearest_index(odom_times, current)]
        np.savetxt(str(pose_dir / f"{idx:06d}.txt"), body_pose @ body_to_optical, fmt="%.12g")

    metric = lambda data: {"count": int(len(data)), "median_m": float(np.median(data)), "p90_m": float(np.percentile(data, 90)), "mean_m": float(np.mean(data))}
    report = {
        "format": "pre_map_vln.real_rgbd_extrinsic.v1",
        "method": "targetless_multiframe_depth_to_concurrent_lidar_scan_nearest_surface_robust_fit",
        "status": "accepted" if np.median(after) < 0.25 and np.median(after) < np.median(before) * 0.90 else "provisional",
        "body_to_camera_link": body_to_link.tolist(),
        "camera_link_to_color_optical": camera_link_to_optical.tolist(),
        "body_to_color_optical": body_to_optical.tolist(),
        "rotation_xyz_deg": Rotation.from_matrix(body_to_link[:3, :3]).as_euler("xyz", degrees=True).tolist(),
        "translation_xyz_m": body_to_link[:3, 3].tolist(),
        "validation_before": metric(before), "validation_after": metric(after),
        "optimizer": {"success": bool(fit.success), "cost": float(fit.cost), "message": fit.message},
        "warning": "Targetless result is valid only while the rigid camera/LiDAR mounting remains unchanged."
    }
    atomic_json(root / "calibration/camera_extrinsic.json", report)
    sha = sha256_file(args.bag)
    manifest = {
        "format": "pre_map_vln.real_boxer_episode.v1", "source_bag": str(args.bag.resolve()),
        "source_bag_sha256": sha, "source_map": str(args.map_pcd.resolve()), "frames": len(selected),
        "first_sec": selected[0], "last_sec": selected[-1], "frame_period_sec": args.frame_period,
        "intrinsics": {"fx": K[0, 0], "fy": K[1, 1], "cx": K[0, 2], "cy": K[1, 2], "width": 640, "height": 480},
        "coordinate_frame": "FAST-LIO world, z-up", "extrinsic_status": report["status"]
    }
    atomic_json(root / "manifest.json", manifest)
    atomic_json(root / "reports/export_report.json", {"status": "ok", "manifest": manifest, "calibration": report})
    print(json.dumps({"output": str(root), "frames": len(selected), "calibration": report}, indent=2))


if __name__ == "__main__":
    main()
