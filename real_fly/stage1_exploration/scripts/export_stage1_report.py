#!/usr/bin/env python3
"""Create a Stage-1 handoff report and trajectory from offline ROS outputs."""

import argparse
import csv
import json
import math
import os
import re
import sys
import time
from collections import OrderedDict

try:
    import rosbag
except ImportError as exc:
    raise SystemExit("rosbag Python module is unavailable; source ROS Noetic first") from exc


UNSAFE = re.compile(
    r"cmd_vel|setpoint|position_command|trajectory|mavros|flight|motor|takeoff|"
    r"offboard|px4|ardupilot|mavlink|habitat|uav_simulator|sensor_pose",
    re.IGNORECASE,
)


def read_env(path):
    values = {}
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def under(root, path):
    try:
        return os.path.commonpath([root, path]) == root
    except ValueError:
        return False


def utc_now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def pcd_header(path):
    header = OrderedDict()
    with open(path, "rb") as handle:
        for _ in range(64):
            raw = handle.readline()
            if not raw:
                break
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("DATA "):
                header["DATA"] = line.split(None, 1)[1]
                break
            if line and not line.startswith("#") and " " in line:
                key, value = line.split(None, 1)
                header[key] = value
    points = header.get("POINTS")
    if points is None:
        width = header.get("WIDTH")
        height = header.get("HEIGHT", "1")
        if width is not None:
            points = str(int(width) * int(height))
    header["points_int"] = int(points) if points and points.isdigit() else 0
    return header


def topic_summary(bag_path):
    topics = OrderedDict()
    unsafe = []
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, bag_time in bag.read_messages():
            if UNSAFE.search(topic):
                unsafe.append(topic)
            info = topics.setdefault(
                topic,
                OrderedDict(
                    type=getattr(message, "_type", message.__class__.__name__),
                    count=0,
                    first=None,
                    last=None,
                    backwards=0,
                    frame_ids=[],
                    child_frame_ids=[],
                ),
            )
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            value = stamp.to_sec() if stamp is not None and stamp.to_sec() > 0 else bag_time.to_sec()
            if info["last"] is not None and value < info["last"]:
                info["backwards"] += 1
            info["count"] += 1
            info["first"] = value if info["first"] is None else min(info["first"], value)
            info["last"] = value if info["last"] is None else max(info["last"], value)
            header = getattr(message, "header", None)
            frame_id = str(getattr(header, "frame_id", ""))
            if frame_id and frame_id not in info["frame_ids"] and len(info["frame_ids"]) < 64:
                info["frame_ids"].append(frame_id)
            for transform in getattr(message, "transforms", []):
                transform_header = getattr(transform, "header", None)
                parent = str(getattr(transform_header, "frame_id", ""))
                child = str(getattr(transform, "child_frame_id", ""))
                if parent and parent not in info["frame_ids"] and len(info["frame_ids"]) < 64:
                    info["frame_ids"].append(parent)
                if child and child not in info["child_frame_ids"] and len(info["child_frame_ids"]) < 64:
                    info["child_frame_ids"].append(child)
    for info in topics.values():
        duration = (info["last"] - info["first"]) if info["first"] is not None else 0.0
        info["duration_sec"] = duration
        info["rate_hz"] = ((info["count"] - 1) / duration) if duration > 0 and info["count"] > 1 else 0.0
    return topics, sorted(set(unsafe))


def extract_trajectory(bag_path, csv_path):
    rows = []
    first_sample = None
    with rosbag.Bag(bag_path, "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=["/Odometry"]):
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            timestamp = stamp.to_sec() if stamp is not None and stamp.to_sec() > 0 else bag_time.to_sec()
            pose = message.pose.pose
            position = pose.position
            orientation = pose.orientation
            values = [position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w]
            if all(math.isfinite(float(value)) for value in values):
                row = [timestamp] + [float(value) for value in values]
                rows.append(row)
                if first_sample is None:
                    first_sample = OrderedDict(
                        timestamp_sec=float(timestamp),
                        frame_id=str(getattr(getattr(message, "header", None), "frame_id", "")),
                        child_frame_id=str(getattr(message, "child_frame_id", "")),
                        position=OrderedDict(x=float(position.x), y=float(position.y), z=float(position.z)),
                        orientation=OrderedDict(
                            x=float(orientation.x),
                            y=float(orientation.y),
                            z=float(orientation.z),
                            w=float(orientation.w),
                        ),
                    )
    with open(csv_path, "x", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(rows)
    path_length = 0.0
    for previous, current in zip(rows, rows[1:]):
        path_length += math.sqrt(sum((current[index] - previous[index]) ** 2 for index in (1, 2, 3)))
    return OrderedDict(samples=len(rows), path_length_m=path_length, csv=csv_path, first_sample=first_sample)


def write_json_new(path, payload):
    with open(path, "x", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def write_text_new(path, payload):
    with open(path, "x", encoding="utf-8") as handle:
        handle.write(payload)


def main():
    stage1_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--raw-bag", required=True)
    parser.add_argument("--mapping-bag", required=True)
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--topics-config", required=True)
    parser.add_argument("--raw-report", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    raw_bag = os.path.abspath(args.raw_bag)
    mapping_bag = os.path.abspath(args.mapping_bag)
    pcd_path = os.path.abspath(args.pcd)
    topics_config = os.path.abspath(args.topics_config)
    raw_report_path = os.path.abspath(args.raw_report)
    output_path = os.path.abspath(args.output)
    for path in (run_dir, raw_bag, mapping_bag, pcd_path, topics_config, raw_report_path, output_path):
        if not under(stage1_root, path):
            raise SystemExit("all Stage-1 inputs and outputs must be under " + stage1_root + ": " + path)
    if not os.path.isdir(run_dir):
        raise SystemExit("run directory not found: " + run_dir)
    for path in (raw_bag, mapping_bag, pcd_path, topics_config):
        if not os.path.isfile(path):
            raise SystemExit("missing required input: " + path)

    expected = read_env(topics_config)
    handoff_dir = os.path.dirname(output_path)
    trajectory_path = os.path.join(handoff_dir, "trajectory.csv")
    start_pose_path = os.path.join(handoff_dir, "start_pose.json")
    frame_description_path = os.path.join(handoff_dir, "frame_description.json")
    sensor_configuration_path = os.path.join(handoff_dir, "sensor_configuration.json")
    calibration_status_path = os.path.join(handoff_dir, "calibration_status.json")
    map_manifest_path = os.path.join(handoff_dir, "map_manifest.json")
    stage2_input_path = os.path.join(handoff_dir, "stage2_input_example.json")
    target_annotations_path = os.path.join(handoff_dir, "target_annotations.todo.json")
    readme_path = os.path.join(handoff_dir, "README.md")
    planned_outputs = [
        output_path,
        trajectory_path,
        start_pose_path,
        frame_description_path,
        sensor_configuration_path,
        calibration_status_path,
        map_manifest_path,
        stage2_input_path,
        target_annotations_path,
        readme_path,
    ]
    if not os.path.exists(raw_report_path):
        planned_outputs.append(raw_report_path)
    conflicts = [path for path in planned_outputs if os.path.exists(path)]
    if conflicts:
        raise SystemExit("refusing to overwrite existing output: " + "; ".join(conflicts))

    os.makedirs(handoff_dir, exist_ok=True)
    raw_topics, raw_unsafe = topic_summary(raw_bag)
    mapping_topics, mapping_unsafe = topic_summary(mapping_bag)
    trajectory = extract_trajectory(mapping_bag, trajectory_path)
    pcd = pcd_header(pcd_path)

    raw_report = OrderedDict(
        bag=raw_bag,
        topics=raw_topics,
        unsafe_topics=raw_unsafe,
        rgb_coverage=OrderedDict(
            verification_level="temporal_stream_only",
            note="Geometric LiDAR-to-RGB projection is not verified by this fallback report.",
        ),
        status="ok" if not raw_unsafe else "failed",
    )
    if os.path.exists(raw_report_path):
        with open(raw_report_path, "r", encoding="utf-8") as handle:
            checked_raw_report = json.load(handle)
    else:
        checked_raw_report = raw_report
        os.makedirs(os.path.dirname(raw_report_path), exist_ok=True)
        write_json_new(raw_report_path, raw_report)

    first_sample = trajectory["first_sample"]
    map_frame = first_sample["frame_id"] if first_sample and first_sample["frame_id"] else "unknown"
    body_frame = first_sample["child_frame_id"] if first_sample and first_sample["child_frame_id"] else "unknown"
    camera_info_topics = [expected.get("RGB_CAMERA_INFO_TOPIC", ""), expected.get("DEPTH_CAMERA_INFO_TOPIC", "")]
    camera_info_recorded = all(topic and topic in raw_topics and raw_topics[topic]["count"] > 0 for topic in camera_info_topics)
    camera_imu_topics = [
        expected.get("CAMERA_IMU_TOPIC", ""),
        expected.get("CAMERA_GYRO_TOPIC", ""),
        expected.get("CAMERA_ACCEL_TOPIC", ""),
    ]
    camera_imu_recorded = any(topic and topic in raw_topics and raw_topics[topic]["count"] > 0 for topic in camera_imu_topics)
    sensor_configuration = OrderedDict(
        topics_config=topics_config,
        expected_topics=expected,
        observed_topics=raw_topics,
        mapping_imu_topic=expected.get("IMU_TOPIC", ""),
        d435i_imu_recorded=camera_imu_recorded,
        notes="Observed topic types and rates come from the raw bag; no control topics are part of this handoff.",
    )
    frame_description = OrderedDict(
        map_frame=map_frame,
        body_frame=body_frame,
        sensor_frame_ids=OrderedDict(
            (topic, info.get("frame_ids", [])) for topic, info in raw_topics.items() if info.get("frame_ids")
        ),
        tf_topics=[topic for topic in ("/tf", "/tf_static") if topic in raw_topics],
        units=OrderedDict(position="meters", timestamp="seconds"),
        note="The map frame is taken from offline FAST-LIO /Odometry when available; verify TF semantics before Stage-2 use.",
    )
    start_pose = OrderedDict(
        status="ok" if first_sample else "pending_no_finite_odometry",
        source_topic="/Odometry",
        pose=first_sample,
        frame_id=map_frame,
    )
    calibration_status = OrderedDict(
        lidar_imu=OrderedDict(
            status="provisional_identity_zero",
            verified=False,
            config=os.path.abspath(os.path.join(stage1_root, "config", "fast_lio_mid360_handheld.yaml")),
        ),
        camera_intrinsics=OrderedDict(
            status="recorded_camera_info" if camera_info_recorded else "missing_or_unverified",
            verified=camera_info_recorded,
            topics=camera_info_topics,
        ),
        lidar_camera_extrinsic=OrderedDict(
            status="missing",
            verified=False,
            required_for_geometric_rgb_coloring=True,
        ),
        time_alignment=OrderedDict(
            status="bag_header_timestamps_only",
            verified=False,
            host_clock_used_for_sensor_sync=False,
        ),
        d435i_imu=OrderedDict(status="recorded" if camera_imu_recorded else "missing_or_unverified", verified=camera_imu_recorded),
    )
    map_manifest = OrderedDict(
        schema_version="stage1_map_manifest_v1",
        generated_utc=utc_now(),
        map_format="PCD",
        map_path=pcd_path,
        point_count=pcd["points_int"],
        coordinate_frame=map_frame,
        units=OrderedDict(position="meters", timestamp="seconds"),
        resolution_m=None,
        origin_xyz=None,
        dimensions_xyz=None,
        source_rosbag=raw_bag,
        mapping_bag=mapping_bag,
        fast_lio_config=os.path.abspath(os.path.join(stage1_root, "config", "fast_lio_mid360_handheld.yaml")),
        occupancy_esdf_conversion="not_generated",
        note="PCD is geometric output only; occupancy/ESDF needs a separately validated conversion.",
    )
    target_annotations = OrderedDict(
        schema_version="stage1_target_annotations_v1",
        status="pending_manual_annotation",
        coordinate_frame=map_frame,
        targets=[],
        note="No target position is fabricated. Add manually reviewed target poses before Stage-2 evaluation.",
    )
    stage2_input = OrderedDict(
        schema_version="stage1_to_stage2_input_example_v1",
        map_pcd=pcd_path,
        map_manifest=map_manifest_path,
        trajectory_csv=trajectory_path,
        start_pose=start_pose_path,
        frame_description=frame_description_path,
        sensor_configuration=sensor_configuration_path,
        calibration_status=calibration_status_path,
        target_annotations=target_annotations_path,
        occupancy_map=None,
        esdf=None,
        live_control_ready=False,
        note="This is an offline handoff example; it does not connect Stage-2 to a flight controller.",
    )

    report = OrderedDict(
        generated_utc=utc_now(),
        run_dir=run_dir,
        raw_bag=raw_bag,
        mapping_bag=mapping_bag,
        handoff_dir=handoff_dir,
        map=OrderedDict(
            path=pcd_path,
            file_size_bytes=os.path.getsize(pcd_path),
            header=pcd,
            status="ok" if pcd["points_int"] > 0 else "failed",
        ),
        trajectory=trajectory,
        mapping_topics=mapping_topics,
        unsafe_topics=sorted(set(raw_unsafe + mapping_unsafe)),
        frame_description=frame_description,
        calibration_status=calibration_status,
        rgb_coverage=OrderedDict(
            verification_level="temporal_stream_only",
            status="pending_manual_extrinsic_projection_check",
            raw_check=checked_raw_report.get("rgb_coverage", {}),
            note="This report verifies recorded RGB/depth stream timing and image payloads; it does not claim geometric point-to-pixel coverage.",
        ),
        stage2_handoff=OrderedDict(
            output_dir=handoff_dir,
            map_manifest=map_manifest_path,
            occupancy_map="PCD only; Stage-2 occupancy/ESDF conversion remains a separate validated step",
            trajectory_csv=trajectory_path,
            start_pose=start_pose_path,
            stage2_input_example=stage2_input_path,
            target_annotations=target_annotations_path,
            live_control_ready=False,
            reason="Stage 1 is sensing/mapping only; no real-flight execution adapter is enabled.",
        ),
        errors=[],
        warnings=[],
    )
    if report["unsafe_topics"]:
        report["errors"].append("unsafe/control-like topic present in input or mapping bag")
    if checked_raw_report.get("status") == "failed":
        report["errors"].append("raw bag quality report failed")
    if "/Odometry" not in mapping_topics:
        report["errors"].append("mapping bag does not contain /Odometry")
    if "/cloud_registered" not in mapping_topics:
        report["warnings"].append("mapping bag does not contain /cloud_registered")
    if trajectory["samples"] < 2:
        report["warnings"].append("fewer than two finite odometry samples")
    if pcd["points_int"] <= 0:
        report["errors"].append("PCD header has no positive point count")
    if not camera_imu_recorded:
        report["warnings"].append("D435i IMU was not confirmed in the raw bag")
    if not camera_info_recorded:
        report["warnings"].append("RGB/depth CameraInfo was not confirmed in the raw bag")
    report["warnings"].append("RGB geometric coloring remains unverified until LiDAR-camera extrinsics and time alignment are measured")
    report["status"] = "ok" if not report["errors"] else "failed"

    write_json_new(start_pose_path, start_pose)
    write_json_new(frame_description_path, frame_description)
    write_json_new(sensor_configuration_path, sensor_configuration)
    write_json_new(calibration_status_path, calibration_status)
    write_json_new(map_manifest_path, map_manifest)
    write_json_new(stage2_input_path, stage2_input)
    write_json_new(target_annotations_path, target_annotations)
    write_text_new(
        readme_path,
        "# Stage-1 handoff\n\n"
        + "Run directory: " + run_dir + "\n\n"
        + "This directory contains offline geometric mapping outputs and metadata.\n"
        + "The PCD map and trajectory are not a live-flight authorization.\n\n"
        + "- Map: `" + pcd_path + "`\n"
        + "- Trajectory: `trajectory.csv`\n"
        + "- RGB status: temporal streams are checked; geometric coloring is not verified.\n"
        + "- Target positions: `target_annotations.todo.json` requires manual annotation.\n"
        + "- Stage-2 live control: disabled.\n",
    )
    write_json_new(output_path, report)
    print(output_path)
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
