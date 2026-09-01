#!/usr/bin/env python3
"""Publish the full-resolution Stage2 map and final trajectory as latched topics."""

import json
import struct

import numpy as np
import rosbag
import rospy
from nav_msgs.msg import Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("rgb", 12, PointField.FLOAT32, 1),
]
FLOOR_COLORS = {
    1: (49, 104, 142),
    2: (68, 1, 84),
    3: (53, 183, 121),
}


def packed_rgb(red, green, blue):
    value = (int(red) << 16) | (int(green) << 8) | int(blue)
    return struct.unpack("f", struct.pack("I", value))[0]


def read_final_messages(bag_path):
    occupied = None
    final_path = None
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, _ in bag.read_messages(
            topics=["/voxel_mapping/occupancy_grid_occupied", "/stage2/executed_path"]
        ):
            if topic == "/voxel_mapping/occupancy_grid_occupied":
                occupied = message
            else:
                final_path = message
    if occupied is None:
        raise RuntimeError("Bag contains no occupied voxel map")
    if final_path is None:
        raise RuntimeError("Bag contains no Stage2 executed path")
    return occupied, final_path


def floor_model(scene_graph, maximum_floor):
    document = json.load(open(scene_graph, encoding="utf-8"))
    floors = sorted(
        (item for item in document.get("floors", []) if int(item["floor_id"]) <= maximum_floor),
        key=lambda item: float(item["floor_z_m"]),
    )
    if not floors:
        raise RuntimeError("scene graph has no usable floors")
    spacings = [
        float(second["floor_z_m"]) - float(first["floor_z_m"])
        for first, second in zip(floors, floors[1:])
    ]
    spacing = float(np.median(spacings)) if spacings else 2.8
    return floors, spacing, float(floors[-1]["floor_z_m"]) + spacing


def point_floor(z_value, floors):
    selected = floors[0]
    for floor in floors:
        if z_value >= float(floor["floor_z_m"]) - 0.25:
            selected = floor
        else:
            break
    return int(selected["floor_id"])


def remove_floor_ceiling(points, floor_z, upper_z, voxel_size):
    """Keep one floor including its slab, but remove the column-wise upper envelope."""
    selected = points[(points[:, 2] >= floor_z - 0.25) & (points[:, 2] < upper_z)]
    if not len(selected):
        return selected
    cells = np.rint(selected[:, :2] / voxel_size).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    column_top = np.full(int(inverse.max()) + 1, -np.inf, dtype=np.float32)
    np.maximum.at(column_top, inverse, selected[:, 2])
    ceiling_columns = column_top[inverse] >= floor_z + 1.65
    ceiling_mask = ceiling_columns & (selected[:, 2] >= column_top[inverse] - 0.45)
    return selected[~ceiling_mask]


def exposed_surface(points, voxel_size):
    """Keep occupied voxels touching at least one non-occupied 6-neighbor."""
    if not len(points):
        return points
    keys = np.rint(points / voxel_size).astype(np.int64)
    occupied = {tuple(key) for key in keys}
    offsets = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
    keep = np.asarray(
        [any(tuple(key + offset) not in occupied for offset in offsets) for key in keys],
        dtype=bool,
    )
    return points[keep]


def main():
    rospy.init_node("stage2_static_map_path")
    bag_path = rospy.get_param("~bag_path")
    scene_graph = rospy.get_param("~scene_graph")
    voxel_size = float(rospy.get_param("~source_voxel_size", 0.10))
    maximum_floor = int(rospy.get_param("~max_floor", 3))
    occupied, final_path = read_final_messages(bag_path)
    floors, _, maximum_z = floor_model(scene_graph, maximum_floor)

    xyz = np.asarray(
        list(point_cloud2.read_points(occupied, field_names=("x", "y", "z"), skip_nans=True)),
        dtype=np.float32,
    )
    xyz = xyz[xyz[:, 2] <= maximum_z]
    source_count = len(xyz)
    surface_xyz = exposed_surface(xyz, voxel_size)

    cloud_rows = []
    for x_value, y_value, z_value in xyz:
        color = FLOOR_COLORS.get(point_floor(float(z_value), floors), (120, 120, 130))
        cloud_rows.append((x_value, y_value, z_value, packed_rgb(*color)))

    stamp = rospy.Time.now()
    header = Header(stamp=stamp, frame_id="world")
    cloud = point_cloud2.create_cloud(header, FIELDS, cloud_rows)
    surface_rows = []
    for x_value, y_value, z_value in surface_xyz:
        color = FLOOR_COLORS.get(point_floor(float(z_value), floors), (120, 120, 130))
        surface_rows.append((x_value, y_value, z_value, packed_rgb(*color)))
    surface_cloud = point_cloud2.create_cloud(header, FIELDS, surface_rows)

    floor_clouds = []
    for index, floor in enumerate(floors):
        floor_id = int(floor["floor_id"])
        floor_z = float(floor["floor_z_m"])
        upper_z = (
            float(floors[index + 1]["floor_z_m"]) - 0.25
            if index + 1 < len(floors) else maximum_z
        )
        visible = remove_floor_ceiling(xyz, floor_z, upper_z, voxel_size)
        color = FLOOR_COLORS.get(floor_id, (120, 120, 130))
        rows = [(x, y, z, packed_rgb(*color)) for x, y, z in visible]
        floor_clouds.append((floor_id, visible.shape[0], point_cloud2.create_cloud(header, FIELDS, rows)))

    final_path.header.stamp = stamp
    final_path.header.frame_id = "world"
    for pose in final_path.poses:
        pose.header.stamp = stamp
        pose.header.frame_id = "world"

    cloud_pub = rospy.Publisher("/stage2/static_floor_cloud", PointCloud2, queue_size=1, latch=True)
    surface_pub = rospy.Publisher("/stage2/static_surface_cloud", PointCloud2, queue_size=1, latch=True)
    path_pub = rospy.Publisher("/stage2/static_complete_path", Path, queue_size=1, latch=True)
    floor_publishers = {
        floor_id: rospy.Publisher(
            "/stage2/static_floor_{}".format(floor_id), PointCloud2, queue_size=1, latch=True
        )
        for floor_id, _, _ in floor_clouds
    }
    rospy.sleep(0.5)
    cloud_pub.publish(cloud)
    surface_pub.publish(surface_cloud)
    path_pub.publish(final_path)
    for floor_id, count, floor_cloud in floor_clouds:
        floor_publishers[floor_id].publish(floor_cloud)
        rospy.loginfo("Static roofless floor %d: %d points", floor_id, count)
    rospy.loginfo(
        "Static Stage2 visualization: %d occupied, %d exposed surface points, path poses %d",
        source_count, len(surface_xyz), len(final_path.poses),
    )
    rospy.spin()


if __name__ == "__main__":
    main()
