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
import yaml
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


TOPICS = {
    "odom": "/Odometry",
    "rgb": "/camera/color/image_raw",
    "depth_aligned": "/camera/aligned_depth_to_color/image_raw",
    "depth_raw": "/camera/depth/image_rect_raw",
    "color_info": "/camera/color/camera_info",
    "depth_info": "/camera/depth/camera_info",
    "depth_to_color": "/camera/extrinsics/depth_to_color",
    "tf_static": "/tf_static",
}


def stamp(msg, bag_time) -> float:
    value = getattr(getattr(getattr(msg, "header", None), "stamp", None), "to_sec", lambda: 0.0)()
    return float(value if value > 0 else bag_time.to_sec())


def rotation_matrix(rotation: Rotation) -> np.ndarray:
    """Support both focal's SciPy 1.3 and current SciPy releases."""
    if hasattr(rotation, "as_matrix"):
        return rotation.as_matrix()
    return rotation.as_dcm()


def rotation_from_matrix(matrix: np.ndarray) -> Rotation:
    if hasattr(Rotation, "from_matrix"):
        return Rotation.from_matrix(matrix)
    return Rotation.from_dcm(matrix)


def pose_matrix(msg) -> np.ndarray:
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation_matrix(Rotation.from_quat([q.x, q.y, q.z, q.w]))
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def transform_matrix(transform) -> np.ndarray:
    p, q = transform.translation, transform.rotation
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation_matrix(Rotation.from_quat([q.x, q.y, q.z, q.w]))
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


def align_depth_to_color(
    depth_mm: np.ndarray,
    depth_K: np.ndarray,
    color_K: np.ndarray,
    color_D: np.ndarray,
    color_shape: tuple[int, int],
    color_from_depth_R: np.ndarray,
    color_from_depth_t: np.ndarray,
) -> np.ndarray:
    """Project rectified depth into the color optical frame with a z-buffer.

    RealSense's Extrinsics message stores its rotation column-major.  The
    caller converts it to a conventional row-major matrix before this helper.
    A 2x2 pixel splat approximates the source-pixel footprint and avoids holes
    caused by the D435 color camera's higher focal length.
    """
    height, width = depth_mm.shape
    vv, uu = np.mgrid[:height, :width]
    z = depth_mm.reshape(-1).astype(np.float64) / 1000.0
    valid = (z > 0.1) & (z < 10.0)
    z = z[valid]
    u = uu.reshape(-1)[valid].astype(np.float64)
    v = vv.reshape(-1)[valid].astype(np.float64)
    points_depth = np.column_stack([
        (u - depth_K[0, 2]) * z / depth_K[0, 0],
        (v - depth_K[1, 2]) * z / depth_K[1, 1],
        z,
    ])
    points_color = points_depth @ color_from_depth_R.T + color_from_depth_t
    in_front = points_color[:, 2] > 0.1
    points_color = points_color[in_front]
    if not len(points_color):
        return np.zeros(color_shape, dtype=np.uint16)
    projected, _ = cv2.projectPoints(
        points_color.reshape(-1, 1, 3),
        np.zeros(3), np.zeros(3), color_K, color_D,
    )
    projected = projected.reshape(-1, 2)
    base = np.floor(projected).astype(np.int32)
    depth_color_mm = np.rint(points_color[:, 2] * 1000.0).astype(np.int64)
    color_height, color_width = color_shape
    zbuffer = np.full(color_height * color_width, np.iinfo(np.uint32).max, dtype=np.uint32)
    for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
        px = base[:, 0] + du
        py = base[:, 1] + dv
        inside = ((px >= 0) & (px < color_width) & (py >= 0) & (py < color_height)
                  & (depth_color_mm > 0) & (depth_color_mm <= np.iinfo(np.uint16).max))
        flat = py[inside] * color_width + px[inside]
        np.minimum.at(zbuffer, flat, depth_color_mm[inside].astype(np.uint32))
    missing = zbuffer == np.iinfo(np.uint32).max
    zbuffer[missing] = 0
    return zbuffer.reshape(color_height, color_width).astype(np.uint16)


def parameter_matrix(values: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation_matrix(Rotation.from_rotvec(values[3:]))
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
    parser.add_argument("--mapping-bag", type=Path,
                        help="bag containing regenerated /Odometry and /cloud_registered")
    parser.add_argument("--pose-bag", type=Path,
                        help="bag containing the pose topic; defaults to mapping bag")
    parser.add_argument("--pose-topic", default="auto",
                        help="pose topic, or auto to prefer /ekf_quat/ekf_odom and fall back to /Odometry")
    parser.add_argument("--frame-period", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=180)
    parser.add_argument("--max-rgb-depth-dt", type=float, default=0.08)
    parser.add_argument("--max-pose-dt", type=float, default=0.08)
    parser.add_argument("--max-cloud-dt", type=float, default=0.10)
    parser.add_argument("--depth-source", choices=("auto", "raw", "aligned"), default="auto",
                        help="auto prefers raw depth plus recorded depth-to-color calibration")
    parser.add_argument("--lidar-camera-extrinsic", type=Path,
                        help="FAST-Calib JSON containing LiDAR-to-color-optical T_cam_lidar")
    parser.add_argument("--fastlio-config", type=Path,
                        help="FAST-LIO YAML containing LiDAR-to-IMU extrinsic_R/extrinsic_T")
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite: {args.output}")
    mapping_bag = args.mapping_bag or args.bag
    pose_bag = args.pose_bag or mapping_bag
    if not args.bag.is_file() or not mapping_bag.is_file() or not pose_bag.is_file() or not args.map_pcd.is_file():
        raise SystemExit("bag or PCD input is missing")

    odom_times, odom_poses, rgb_times, depth_times = [], [], [], []
    color_K, depth_K, color_D = None, None, None
    color_from_depth_R, color_from_depth_t = None, None
    camera_link_to_optical = None
    with rosbag.Bag(str(pose_bag), "r") as bag:
        available_pose_topics = set(bag.get_type_and_topic_info()[1])
        if args.pose_topic == "auto":
            pose_topic = "/ekf_quat/ekf_odom" if "/ekf_quat/ekf_odom" in available_pose_topics else TOPICS["odom"]
        else:
            pose_topic = args.pose_topic
        if pose_topic not in available_pose_topics:
            raise SystemExit(f"selected pose topic is missing: {pose_topic}")
        for _, msg, bt in bag.read_messages(topics=[pose_topic]):
            odom_times.append(stamp(msg, bt)); odom_poses.append(pose_matrix(msg))
    if not odom_times:
        raise SystemExit(f"pose topic has no messages: {pose_topic}")
    with rosbag.Bag(str(args.bag), "r") as bag:
        available_topics = set(bag.get_type_and_topic_info()[1])
        raw_ready = all(TOPICS[name] in available_topics for name in
                        ("depth_raw", "depth_info", "color_info", "depth_to_color"))
        if args.depth_source == "raw" and not raw_ready:
            raise SystemExit("raw depth requested but raw depth calibration topics are missing")
        use_raw_depth = raw_ready if args.depth_source == "auto" else args.depth_source == "raw"
        source_depth_topic = TOPICS["depth_raw"] if use_raw_depth else TOPICS["depth_aligned"]
        if source_depth_topic not in available_topics:
            raise SystemExit(f"selected depth topic is missing: {source_depth_topic}")
        source_topics = [TOPICS["rgb"], source_depth_topic, TOPICS["color_info"],
                         TOPICS["depth_info"], TOPICS["depth_to_color"], TOPICS["tf_static"]]
        first_tf, second_tf = None, None
        for topic, msg, bt in bag.read_messages(topics=source_topics):
            current = stamp(msg, bt)
            if topic == TOPICS["rgb"]: rgb_times.append(current)
            elif topic == source_depth_topic: depth_times.append(current)
            elif topic == TOPICS["color_info"] and color_K is None:
                color_K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
                color_D = np.asarray(msg.D, dtype=np.float64)
                color_shape = (int(msg.height), int(msg.width))
            elif topic == TOPICS["depth_info"] and depth_K is None:
                depth_K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
            elif topic == TOPICS["depth_to_color"] and color_from_depth_R is None:
                # librealsense rs2_extrinsics.rotation is serialized column-major.
                color_from_depth_R = np.asarray(msg.rotation, dtype=np.float64).reshape(3, 3).T
                color_from_depth_t = np.asarray(msg.translation, dtype=np.float64)
            elif topic == TOPICS["tf_static"]:
                for item in msg.transforms:
                    if item.header.frame_id == "camera_link" and item.child_frame_id == "camera_aligned_depth_to_color_frame":
                        first_tf = transform_matrix(item.transform)
                    elif item.header.frame_id == "camera_aligned_depth_to_color_frame" and item.child_frame_id == "camera_color_optical_frame":
                        second_tf = transform_matrix(item.transform)
                if first_tf is not None and second_tf is not None:
                    camera_link_to_optical = first_tf @ second_tf
    if not odom_times or not rgb_times or not depth_times or color_K is None:
        raise SystemExit("required odometry/RGB/depth/CameraInfo stream is missing")
    if use_raw_depth and any(value is None for value in
                             (depth_K, color_D, color_from_depth_R, color_from_depth_t)):
        raise SystemExit("raw depth selected but intrinsics/extrinsics could not be decoded")
    if camera_link_to_optical is None:
        camera_link_to_optical = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)

    valid_pairs = []
    rejected_alignment = {"outside_pose_coverage": 0, "rgb_depth_dt": 0, "pose_dt": 0}
    # RGB is the semantic inference clock. Select RGB keyframes first, then
    # attach the nearest raw/aligned depth and pose; selecting on the faster
    # depth stream can unnecessarily retain a poor RGB-depth pairing.
    for rgb_time in rgb_times:
        depth_time = depth_times[nearest_index(depth_times, rgb_time)]
        if not odom_times[0] <= rgb_time <= odom_times[-1]:
            rejected_alignment["outside_pose_coverage"] += 1
            continue
        rgb_dt = abs(rgb_time - depth_time)
        pose_dt = abs(odom_times[nearest_index(odom_times, rgb_time)] - rgb_time)
        if rgb_dt > args.max_rgb_depth_dt:
            rejected_alignment["rgb_depth_dt"] += 1
            continue
        if pose_dt > args.max_pose_dt:
            rejected_alignment["pose_dt"] += 1
            continue
        valid_pairs.append((rgb_time, depth_time))
    selected_pairs, last = [], -math.inf
    for rgb_time, depth_time in valid_pairs:
        if rgb_time - last >= args.frame_period:
            selected_pairs.append((rgb_time, depth_time)); last = rgb_time
        if len(selected_pairs) >= args.max_frames: break
    selected_rgb = [item[0] for item in selected_pairs]
    selected = [item[1] for item in selected_pairs]
    if not selected:
        raise SystemExit("no RGB/depth/odometry triples satisfy the timestamp limits")
    depth_lookup = {round(value, 6): idx for idx, value in enumerate(selected)}
    rgb_lookup = {round(value, 6): idx for idx, value in enumerate(selected_rgb)}

    root = args.output
    color_dir, depth_dir, pose_dir = root / "frames/color", root / "frames/depth", root / "frames/pose"
    intrinsic_dir = root / "frames/intrinsic"
    for path in (color_dir, depth_dir, pose_dir, intrinsic_dir, root / "calibration", root / "reports"):
        path.mkdir(parents=True, exist_ok=False)
    depth_arrays = [None] * len(selected)
    rgb_arrays = [None] * len(selected)
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, bt in bag.read_messages(topics=[TOPICS["rgb"], source_depth_topic]):
            key = round(stamp(msg, bt), 6)
            if topic == TOPICS["rgb"] and key in rgb_lookup:
                idx = rgb_lookup[key]; image = image_array(msg)
                if msg.encoding == "rgb8":
                    rgb_image = image
                    jpeg_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                elif msg.encoding == "bgr8":
                    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    jpeg_image = image
                else:
                    raise SystemExit(f"unsupported RGB encoding: {msg.encoding}")
                rgb_arrays[idx] = np.ascontiguousarray(rgb_image)
                cv2.imwrite(str(color_dir / f"{idx:06d}.jpg"), jpeg_image, [cv2.IMWRITE_JPEG_QUALITY, 94])
            elif topic == source_depth_topic and key in depth_lookup:
                idx = depth_lookup[key]; image = image_array(msg).astype(np.uint16)
                if use_raw_depth:
                    image = align_depth_to_color(
                        image, depth_K, color_K, color_D, color_shape,
                        color_from_depth_R, color_from_depth_t,
                    )
                depth_arrays[idx] = image
                cv2.imwrite(str(depth_dir / f"{idx:06d}.png"), image)

    calibration_samples = []
    for idx in range(0, len(selected), max(1, len(selected) // 30)):
        if depth_arrays[idx] is None: continue
        points = depth_points(depth_arrays[idx], color_K, stride=24)
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
    with rosbag.Bag(str(mapping_bag), "r") as bag:
        for _, msg, bt in bag.read_messages(topics=["/cloud_registered"]):
            current = stamp(msg, bt)
            idx = nearest_index(sample_times, current)
            delta = abs(sample_times[idx] - current)
            if delta < best_dt[idx] and delta <= args.max_cloud_dt:
                points = cloud_xyz(msg)
                points = voxel_downsample(points[np.isfinite(points).all(axis=1)], 0.08)
                calibration_samples[idx][3] = cKDTree(points)
                best_dt[idx] = delta
    calibration_samples = [item for item in calibration_samples if item[3] is not None]
    if len(calibration_samples) < 8:
        raise SystemExit("too few time-paired depth/LiDAR scans for targetless calibration")

    def residual_transform(body_to_optical, samples):
        result = []
        for _, body_pose, points, tree in samples:
            world = (body_pose @ body_to_optical @ np.column_stack([points, np.ones(len(points))]).T).T[:, :3]
            # SciPy 1.3 in the ROS Noetic image predates the workers keyword.
            distances = tree.query(world, k=1)[0]
            result.append(np.minimum(distances, 0.75))
        return np.concatenate(result)

    def residual(values, samples):
        return residual_transform(parameter_matrix(values) @ camera_link_to_optical, samples)

    train = calibration_samples[::2]
    validation = calibration_samples[1::2] or train
    initial = np.zeros(6, dtype=np.float64)
    before = residual(initial, validation)
    if args.lidar_camera_extrinsic:
        if not args.fastlio_config:
            raise SystemExit("--fastlio-config is required with --lidar-camera-extrinsic")
        calibrated = json.loads(args.lidar_camera_extrinsic.read_text(encoding="utf-8"))
        if calibrated.get("direction") != "lidar_to_camera_optical":
            raise SystemExit("calibration direction must be lidar_to_camera_optical")
        fastlio = yaml.safe_load(args.fastlio_config.read_text(encoding="utf-8"))
        mapping = fastlio["mapping"]
        imu_to_lidar = np.eye(4, dtype=np.float64)
        imu_to_lidar[:3, :3] = np.asarray(mapping["extrinsic_R"], dtype=np.float64).reshape(3, 3)
        imu_to_lidar[:3, 3] = np.asarray(mapping["extrinsic_T"], dtype=np.float64)
        camera_to_lidar = np.eye(4, dtype=np.float64)
        camera_to_lidar[:3, :3] = np.asarray(calibrated["R_cam_lidar"], dtype=np.float64)
        camera_to_lidar[:3, 3] = np.asarray(calibrated["P_cam_lidar_m"], dtype=np.float64)
        # FAST-LIO stores T_imu_lidar, while FAST-Calib reports T_camera_lidar.
        # Camera points reach the FAST-LIO body frame through T_imu_lidar * T_lidar_camera.
        body_to_optical = imu_to_lidar @ np.linalg.inv(camera_to_lidar)
        after = residual_transform(body_to_optical, validation)
        body_to_link = body_to_optical @ np.linalg.inv(camera_link_to_optical)
        fit = None
        method = "FAST-Calib_target_three_scene_composed_with_FAST-LIO_lidar_to_IMU"
        status = "accepted"
    else:
        fit = least_squares(residual, initial, args=(train,), loss="soft_l1", f_scale=0.12,
                            bounds=([-0.25] * 3 + [-0.40] * 3, [0.25] * 3 + [0.40] * 3), max_nfev=80, verbose=1)
        after = residual(fit.x, validation)
        body_to_link = parameter_matrix(fit.x)
        body_to_optical = body_to_link @ camera_link_to_optical
        method = "targetless_multiframe_depth_to_concurrent_lidar_scan_nearest_surface_robust_fit"
        status = "accepted" if np.median(after) < 0.25 and np.median(after) < np.median(before) * 0.90 else "provisional"

    intrinsic = np.eye(4); intrinsic[:3, :3] = color_K
    np.savetxt(str(intrinsic_dir / "intrinsic_color.txt"), intrinsic, fmt="%.12g")
    np.savetxt(str(intrinsic_dir / "intrinsic_depth.txt"), intrinsic, fmt="%.12g")
    frame_records = []
    for idx, current in enumerate(selected):
        if depth_arrays[idx] is None or rgb_arrays[idx] is None:
            raise SystemExit(f"selected frame {idx} was not decoded from the source bag")
        odom_idx = nearest_index(odom_times, current)
        optical_pose = odom_poses[odom_idx] @ body_to_optical
        np.savetxt(str(pose_dir / f"{idx:06d}.txt"), optical_pose, fmt="%.12g")
        rgb_time = selected_rgb[idx]
        time_ns = int(round(rgb_time * 1e9))
        orientation_xyzw = rotation_from_matrix(optical_pose[:3, :3]).as_quat().astype(np.float32)
        np.savez_compressed(
            str(root / f"frame_{idx:06d}.npz"),
            rgb=rgb_arrays[idx],
            depth_m=depth_arrays[idx].astype(np.float32) / 1000.0,
            position=optical_pose[:3, 3].astype(np.float32),
            orientation_xyzw=orientation_xyzw,
            time_ns=np.int64(time_ns),
            rgb_time_sec=np.float64(rgb_time),
            depth_time_sec=np.float64(current),
            odometry_time_sec=np.float64(odom_times[odom_idx]),
        )
        frame_records.append({
            "index": idx,
            "time_ns": time_ns,
            "rgb_time_sec": rgb_time,
            "depth_time_sec": current,
            "odometry_time_sec": odom_times[odom_idx],
            "rgb_depth_dt_sec": abs(rgb_time - current),
            "depth_odometry_dt_sec": abs(odom_times[odom_idx] - current),
        })

    metric = lambda data: {"count": int(len(data)), "median_m": float(np.median(data)), "p90_m": float(np.percentile(data, 90)), "mean_m": float(np.mean(data))}
    report = {
        "format": "pre_map_vln.real_rgbd_extrinsic.v1",
        "method": method,
        "status": status,
        "body_to_camera_link": body_to_link.tolist(),
        "camera_link_to_color_optical": camera_link_to_optical.tolist(),
        "body_to_color_optical": body_to_optical.tolist(),
        "rotation_xyz_deg": rotation_from_matrix(body_to_link[:3, :3]).as_euler("xyz", degrees=True).tolist(),
        "translation_xyz_m": body_to_link[:3, 3].tolist(),
        "validation_before": metric(before), "validation_after": metric(after),
        "optimizer": ({"success": bool(fit.success), "cost": float(fit.cost), "message": fit.message}
                      if fit is not None else None),
        "lidar_camera_calibration": calibrated if args.lidar_camera_extrinsic else None,
        "fastlio_lidar_imu_config": str(args.fastlio_config.resolve()) if args.fastlio_config else None,
        "depth_source": "raw_offline_aligned_to_color" if use_raw_depth else "realtime_aligned_depth",
        "depth_source_topic": source_depth_topic,
        "depth_to_color_rotation": color_from_depth_R.tolist() if use_raw_depth else None,
        "depth_to_color_translation_m": color_from_depth_t.tolist() if use_raw_depth else None,
        "warning": "Calibration is valid only while the rigid camera/LiDAR mounting remains unchanged."
    }
    atomic_json(root / "calibration/camera_extrinsic.json", report)
    sha = sha256_file(args.bag)
    mapping_sha = sha if mapping_bag.resolve() == args.bag.resolve() else sha256_file(mapping_bag)
    rgb_depth_errors = [item["rgb_depth_dt_sec"] for item in frame_records]
    pose_errors = [item["depth_odometry_dt_sec"] for item in frame_records]
    manifest = {
        "format": "pre_map_vln.real_boxer_episode.v2", "source_bag": str(args.bag.resolve()),
        "source_bag_sha256": sha, "mapping_bag": str(mapping_bag.resolve()),
        "pose_bag": str(pose_bag.resolve()), "pose_topic": pose_topic,
        "mapping_bag_sha256": mapping_sha, "source_map": str(args.map_pcd.resolve()), "frames": len(selected),
        "first_sec": selected[0], "last_sec": selected[-1], "frame_period_sec": args.frame_period,
        "fx": float(color_K[0, 0]), "fy": float(color_K[1, 1]), "cx": float(color_K[0, 2]), "cy": float(color_K[1, 2]),
        "intrinsics": {"fx": color_K[0, 0], "fy": color_K[1, 1], "cx": color_K[0, 2], "cy": color_K[1, 2],
                       "width": color_shape[1], "height": color_shape[0]},
        "depth_source": report["depth_source"],
        "depth_source_topic": source_depth_topic,
        "raw_depth_intrinsics": ({"K": depth_K.tolist()} if use_raw_depth else None),
        "coordinate_frame": "FAST-LIO world, z-up", "extrinsic_status": report["status"],
        "alignment": {
            "max_rgb_depth_dt_sec": max(rgb_depth_errors),
            "max_depth_odometry_dt_sec": max(pose_errors),
            "limits_sec": {"rgb_depth": args.max_rgb_depth_dt, "depth_odometry": args.max_pose_dt,
                           "depth_cloud": args.max_cloud_dt},
            "rejected_depth_frames": rejected_alignment,
        },
        "frame_records": frame_records,
    }
    atomic_json(root / "manifest.json", manifest)
    atomic_json(root / "reports/export_report.json", {"status": "ok", "manifest": manifest, "calibration": report})
    print(json.dumps({"output": str(root), "frames": len(selected), "calibration": report}, indent=2))


if __name__ == "__main__":
    main()
