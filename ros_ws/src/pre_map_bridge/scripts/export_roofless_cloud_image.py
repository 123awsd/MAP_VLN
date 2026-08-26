#!/usr/bin/env python3
"""Export the final explored cloud in a reproducible roofless isometric view."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import PointField


CLOUD_TOPIC = "/voxel_mapping/occupancy_grid_occupied"
ODOM_TOPIC = "/uav_simulator/odometry"


def xyz_from_cloud(message):
    fields = {field.name: field for field in message.fields if field.name in {"x", "y", "z"}}
    if set(fields) != {"x", "y", "z"}:
        raise ValueError("cloud does not contain XYZ fields")
    if any(field.datatype != PointField.FLOAT32 for field in fields.values()):
        raise ValueError("cloud XYZ fields must use FLOAT32")
    count = len(message.data) // message.point_step
    endian = ">f4" if message.is_bigendian else "<f4"
    values = [
        np.ndarray(
            (count,), dtype=endian, buffer=message.data,
            offset=fields[axis].offset, strides=(message.point_step,),
        ).astype(np.float32, copy=True)
        for axis in ("x", "y", "z")
    ]
    points = np.column_stack(values)
    return points[np.isfinite(points).all(axis=1)]


def roofless(points, floor_z, floor_clearance, ceiling_min_height, ceiling_thickness, voxel):
    x_cells = np.rint(points[:, 0] / voxel).astype(np.int64)
    y_cells = np.rint(points[:, 1] / voxel).astype(np.int64)
    y_min = int(y_cells.min())
    y_span = int(y_cells.max()) - y_min + 1
    keys = (x_cells - int(x_cells.min())) * y_span + (y_cells - y_min)
    _, inverse = np.unique(keys, return_inverse=True)
    top = np.full(int(inverse.max()) + 1, -np.inf, dtype=np.float32)
    np.maximum.at(top, inverse, points[:, 2])
    floor = points[:, 2] <= floor_z + floor_clearance
    ceiling_column = top[inverse] >= floor_z + ceiling_min_height
    ceiling = ceiling_column & (points[:, 2] >= top[inverse] - ceiling_thickness)
    return points[~(floor | ceiling)]


def colors_for_height(z, floor_z, floor_clearance, ceiling_min_height):
    ratio = np.clip(
        (z - floor_z - floor_clearance)
        / max(0.1, ceiling_min_height - floor_clearance), 0.0, 1.0,
    )
    stops = np.asarray([0.0, 0.55, 1.0])
    # BGR equivalent of the restrained cyan-blue-magenta RViz palette.
    blue = np.interp(ratio, stops, [230, 210, 210])
    green = np.interp(ratio, stops, [210, 70, 0])
    red = np.interp(ratio, stops, [0, 35, 235])
    return np.column_stack([blue, green, red]).astype(np.uint8)


def render(points, colors, width, height, azimuth_deg, elevation_deg, point_radius):
    center = 0.5 * (points.min(axis=0) + points.max(axis=0))
    centered = points - center
    azimuth = np.deg2rad(azimuth_deg)
    elevation = np.deg2rad(elevation_deg)
    screen_x = -np.sin(azimuth) * centered[:, 0] + np.cos(azimuth) * centered[:, 1]
    screen_y = (
        -np.sin(elevation) * np.cos(azimuth) * centered[:, 0]
        - np.sin(elevation) * np.sin(azimuth) * centered[:, 1]
        + np.cos(elevation) * centered[:, 2]
    )
    depth = (
        np.cos(elevation) * np.cos(azimuth) * centered[:, 0]
        + np.cos(elevation) * np.sin(azimuth) * centered[:, 1]
        + np.sin(elevation) * centered[:, 2]
    )
    margin = 70
    span_x = max(float(np.ptp(screen_x)), 1e-6)
    span_y = max(float(np.ptp(screen_y)), 1e-6)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)
    pixel_x = np.rint((screen_x - screen_x.min()) * scale + margin).astype(np.int32)
    pixel_y = np.rint((screen_y.max() - screen_y) * scale + margin).astype(np.int32)
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    # Far surfaces first, then near surfaces. A small radius keeps the visual
    # recognisably point-cloud based while closing only sub-pixel holes.
    for index in np.argsort(depth):
        cv2.circle(
            image, (int(pixel_x[index]), int(pixel_y[index])), point_radius,
            tuple(int(value) for value in colors[index]), -1, lineType=cv2.LINE_AA,
        )
    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-clearance", type=float, default=0.25)
    parser.add_argument("--ceiling-min-height", type=float, default=1.65)
    parser.add_argument("--ceiling-thickness", type=float, default=0.60)
    parser.add_argument("--voxel-resolution", type=float, default=0.10)
    parser.add_argument("--sensor-height", type=float, default=1.0)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1440)
    parser.add_argument("--azimuth", type=float, default=-42.0)
    parser.add_argument("--elevation", type=float, default=58.0)
    parser.add_argument("--point-radius", type=int, default=1)
    args = parser.parse_args()

    last_cloud = None
    last_sensor_z = None
    cloud_messages = 0
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, _ in bag.read_messages(topics=[CLOUD_TOPIC, ODOM_TOPIC]):
            if topic == CLOUD_TOPIC:
                last_cloud = message
                cloud_messages += 1
            else:
                last_sensor_z = float(message.pose.pose.position.z)
    if last_cloud is None:
        raise RuntimeError("bag contains no final occupancy point cloud")
    points = xyz_from_cloud(last_cloud)
    if not len(points):
        raise RuntimeError("final occupancy point cloud is empty")
    # Match RViz: UAV sensor is nominally 1 m above the selected floor. Fall
    # back to a conservative low percentile only for legacy bags without odom.
    floor_z = (
        last_sensor_z - args.sensor_height
        if last_sensor_z is not None else float(np.percentile(points[:, 2], 1.0))
    )
    visible = roofless(
        points, floor_z, args.floor_clearance, args.ceiling_min_height,
        args.ceiling_thickness, args.voxel_resolution,
    )
    if not len(visible):
        raise RuntimeError("roofless filter removed every point")
    colors = colors_for_height(
        visible[:, 2], floor_z, args.floor_clearance, args.ceiling_min_height
    )
    image = render(
        visible, colors, args.width, args.height, args.azimuth,
        args.elevation, max(1, args.point_radius),
    )
    title = "{}  |  explored roofless point cloud".format(args.scene_id)
    cv2.putText(image, title, (42, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (45, 45, 45), 2, cv2.LINE_AA)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), image):
        raise RuntimeError("failed to write {}".format(args.output))
    metadata = {
        "format": "pre_map_vln.roofless_cloud_image.v1",
        "scene_id": args.scene_id,
        "source_bag": str(args.bag),
        "cloud_topic": CLOUD_TOPIC,
        "cloud_message_count": cloud_messages,
        "source_point_count": int(len(points)),
        "visible_point_count": int(len(visible)),
        "floor_z_m": floor_z,
        "floor_clearance_m": args.floor_clearance,
        "ceiling_min_height_m": args.ceiling_min_height,
        "ceiling_thickness_m": args.ceiling_thickness,
        "azimuth_deg": args.azimuth,
        "elevation_deg": args.elevation,
        "image": str(args.output),
    }
    metadata_path = args.metadata or args.output.with_suffix(".json")
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(metadata, ensure_ascii=False))


if __name__ == "__main__":
    main()
