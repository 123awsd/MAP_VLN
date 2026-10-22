#!/usr/bin/env python3
"""Inject cumulative Boxer MarkerArrays into a bag at each instance first-seen time."""

import argparse
import csv
from pathlib import Path

import rosbag
import rospy
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


PALETTE = [
    (0.12, 0.72, 1.00), (1.00, 0.45, 0.12), (0.45, 0.90, 0.25),
    (0.85, 0.30, 0.90), (1.00, 0.82, 0.15), (0.25, 0.85, 0.75),
]


def marker_array(rows, stamp):
    markers = MarkerArray()
    clear = Marker()
    clear.header.stamp, clear.header.frame_id = stamp, "world"
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)
    for index, row in enumerate(rows):
        red, green, blue = PALETTE[index % len(PALETTE)]
        cube = Marker()
        cube.header.stamp, cube.header.frame_id = stamp, "world"
        cube.ns, cube.id, cube.type, cube.action = "boxer_3d", index, Marker.CUBE, Marker.ADD
        cube.pose.position.x = float(row["tx_world_object"])
        cube.pose.position.y = float(row["ty_world_object"])
        cube.pose.position.z = float(row["tz_world_object"])
        cube.pose.orientation.w = float(row["qw_world_object"])
        cube.pose.orientation.x = float(row["qx_world_object"])
        cube.pose.orientation.y = float(row["qy_world_object"])
        cube.pose.orientation.z = float(row["qz_world_object"])
        cube.scale.x = float(row["scale_x"])
        cube.scale.y = float(row["scale_y"])
        cube.scale.z = float(row["scale_z"])
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = red, green, blue, 0.025
        markers.markers.append(cube)

        outline = Marker()
        outline.header.stamp, outline.header.frame_id = stamp, "world"
        outline.ns, outline.id = "boxer_outlines", index
        outline.type, outline.action, outline.pose = Marker.LINE_LIST, Marker.ADD, cube.pose
        outline.scale.x = 0.012
        outline.color.r, outline.color.g, outline.color.b, outline.color.a = red, green, blue, 0.45
        hx, hy, hz = cube.scale.x * 0.5, cube.scale.y * 0.5, cube.scale.z * 0.5
        corners = [
            (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
            (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
        ]
        for start, end in [
            (0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6),
            (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7),
        ]:
            outline.points.extend([Point(*corners[start]), Point(*corners[end])])
        markers.markers.append(outline)

        label = Marker()
        label.header.stamp, label.header.frame_id = stamp, "world"
        label.ns, label.id, label.type, label.action = "boxer_labels", index, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x = cube.pose.position.x
        label.pose.position.y = cube.pose.position.y
        label.pose.position.z = cube.pose.position.z + cube.scale.z * 0.5 + 0.2
        label.pose.orientation.w = 1.0
        label.scale.z = 0.24
        label.color.r, label.color.g, label.color.b, label.color.a = red, green, blue, 0.78
        label.text = "%s %.2f" % (row["name"], float(row["prob"]))
        markers.markers.append(label)
    return markers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("box_csv", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument(
        "--max-map-distance",
        type=float,
        default=0.75,
        help="discard visualization boxes farther than this many meters from the final occupied cloud",
    )
    args = parser.parse_args()
    if args.output_bag.exists():
        raise FileExistsError(args.output_bag)

    with args.box_csv.open(newline="", encoding="utf-8") as stream:
        rows = sorted(csv.DictReader(stream), key=lambda row: int(row["first_seen_ns"]))
    events = []
    for row in rows:
        timestamp = int(row["first_seen_ns"])
        if not events or events[-1][0] != timestamp:
            events.append((timestamp, []))
        events[-1][1].append(row)

    last_occupied = None
    with rosbag.Bag(str(args.input_bag), "r") as source:
        rgb_stamps = [
            stamp.to_nsec()
            for _, _, stamp in source.read_messages(topics=["/habitat/rgb"])
        ]
        for _, message, _ in source.read_messages(
            topics=["/voxel_mapping/occupancy_grid_occupied"]
        ):
            last_occupied = message
    if not rgb_stamps:
        raise RuntimeError("Input bag contains no /habitat/rgb frames")
    if args.max_map_distance >= 0 and last_occupied is not None:
        occupied = np.asarray(
            list(
                point_cloud2.read_points(
                    last_occupied, field_names=("x", "y", "z"), skip_nans=True
                )
            ),
            dtype=np.float64,
        )
        if occupied.size == 0:
            raise RuntimeError("Final occupied cloud is empty; cannot filter boxes")
        kept, rejected = [], []
        for row in rows:
            center = np.asarray(
                [
                    float(row["tx_world_object"]),
                    float(row["ty_world_object"]),
                    float(row["tz_world_object"]),
                ]
            )
            distance = float(np.sqrt(np.square(occupied - center).sum(axis=1)).min())
            (kept if distance <= args.max_map_distance else rejected).append(row)
        rows = kept
        events = []
        for row in rows:
            timestamp = int(row["first_seen_ns"])
            if not events or events[-1][0] != timestamp:
                events.append((timestamp, []))
            events[-1][1].append(row)
        if rejected:
            print(
                "map-distance filter rejected %d visualization boxes: %s"
                % (len(rejected), ", ".join(row["name"] for row in rejected))
            )
    # Make the RGB stream define the replay interval.  This guarantees that
    # every visible exploration timestamp has a corresponding first-person
    # frame instead of leaving point-cloud-only margins at either end.
    crop_start_ns = rgb_stamps[0]
    crop_end_ns = rgb_stamps[-1]

    visible, event_index = [], 0
    with rosbag.Bag(str(args.input_bag), "r") as source, rosbag.Bag(
        str(args.output_bag), "w", compression=rosbag.Compression.LZ4
    ) as target:
        for topic, message, stamp in source.read_messages():
            if topic == "/pre_map_vln/box_markers":
                continue
            stamp_ns = stamp.to_nsec()
            if stamp_ns < crop_start_ns or stamp_ns > crop_end_ns:
                continue
            # FALCON's executed trajectory is a SPHERE_LIST.  Its upstream
            # 0.10 m spheres look like a thick dashed tube in this small scene.
            if topic == "/planning/travel_traj" and message.type == Marker.SPHERE_LIST:
                message.scale.x = min(message.scale.x, 0.025)
                message.scale.y = min(message.scale.y, 0.025)
                message.scale.z = min(message.scale.z, 0.025)
                message.color.a = min(message.color.a, 0.58)
            while event_index < len(events) and events[event_index][0] <= stamp_ns:
                event_ns, additions = events[event_index]
                visible.extend(additions)
                event_stamp = rospy.Time.from_sec(max(event_ns, crop_start_ns) / 1e9)
                target.write(
                    "/pre_map_vln/box_markers",
                    marker_array(visible, event_stamp),
                    event_stamp,
                )
                event_index += 1
            target.write(topic, message, stamp)
    print(
        "cropped to %.3fs; injected %d progressive box events containing %d final instances"
        % ((crop_end_ns - crop_start_ns) / 1e9, event_index, len(visible))
    )


if __name__ == "__main__":
    main()
