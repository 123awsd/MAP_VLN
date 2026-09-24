#!/usr/bin/env python3
"""Publish reconstructed PLY maps as latched ROS 2 PointCloud2 topics."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


PLY_TYPES = {
    "char": "i1",
    "uchar": "u1",
    "short": "<i2",
    "ushort": "<u2",
    "int": "<i4",
    "uint": "<u4",
    "float": "<f4",
    "double": "<f8",
}


def read_binary_ply(path: Path) -> np.ndarray:
    properties: list[tuple[str, str]] = []
    vertex_count = None
    with path.open("rb") as stream:
        if stream.readline() != b"ply\n":
            raise ValueError(f"Not a PLY file: {path}")
        if stream.readline().strip() != b"format binary_little_endian 1.0":
            raise ValueError(f"Only binary little-endian PLY is supported: {path}")
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PLY header has no end_header: {path}")
            words = line.decode("ascii").strip().split()
            if words[:2] == ["element", "vertex"]:
                vertex_count = int(words[2])
            elif words[:1] == ["property"] and len(words) == 3:
                properties.append((words[2], PLY_TYPES[words[1]]))
            elif words[:1] == ["end_header"]:
                data_offset = stream.tell()
                break
    if vertex_count is None:
        raise ValueError(f"PLY has no vertex element: {path}")
    required = {"x", "y", "z", "red", "green", "blue"}
    names = {name for name, _ in properties}
    if not required <= names:
        raise ValueError(f"PLY lacks required fields {sorted(required - names)}: {path}")
    return np.memmap(
        path, dtype=np.dtype(properties), mode="r", offset=data_offset, shape=(vertex_count,)
    )


def make_cloud(
    path: Path, frame_id: str, *, exclude_labels: set[int] | None = None
) -> PointCloud2:
    source = read_binary_ply(path)
    if exclude_labels:
        if "label" not in source.dtype.names:
            raise ValueError(f"Cannot filter labels from a PLY without a label field: {path}")
        source = source[~np.isin(source["label"], list(exclude_labels))]
    packed = np.empty(
        len(source),
        dtype=np.dtype(
            {
                "names": ["x", "y", "z", "rgb"],
                "formats": ["<f4", "<f4", "<f4", "<u4"],
                "offsets": [0, 4, 8, 12],
                "itemsize": 16,
            }
        ),
    )
    packed["x"] = source["x"]
    packed["y"] = source["y"]
    packed["z"] = source["z"]
    packed["rgb"] = (
        source["red"].astype(np.uint32) << 16
        | source["green"].astype(np.uint32) << 8
        | source["blue"].astype(np.uint32)
    )

    message = PointCloud2()
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(packed)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        # PCL/RViz convention: packed 0x00RRGGBB bits carried in a float field.
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = packed.dtype.itemsize
    message.row_step = message.point_step * message.width
    message.data = packed.tobytes()
    message.is_dense = False
    return message


def make_occupancy_grid(path: Path, frame_id: str) -> OccupancyGrid:
    archive = np.load(path, allow_pickle=False)
    grid = archive["occupancy"].astype(np.int8, copy=False)
    resolution = float(archive["resolution"])
    origin = archive["origin"]
    message = OccupancyGrid()
    message.header.frame_id = frame_id
    message.info.resolution = resolution
    message.info.width = grid.shape[1]
    message.info.height = grid.shape[0]
    message.info.origin.position.x = float(origin[0])
    message.info.origin.position.y = float(origin[1])
    message.info.origin.position.z = float(origin[2])
    message.info.origin.orientation.w = 1.0
    message.data = grid.ravel().tolist()
    return message


class ScenePublisher(Node):
    def __init__(self, scene_dir: Path, frame_id: str) -> None:
        super().__init__("reconstructed_scene_publisher")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        inputs = {
            "/reconstruction/roofless_height_gradient": (
                "map_roofless_height_gradient_5cm.ply",
                None,
            ),
            "/reconstruction/occupancy_3d": (
                "occupancy_3d/occupied_voxels_10cm.ply",
                None,
            ),
            "/reconstruction/roofless_rgb": ("map_rgb_roofless_clean_5cm.ply", None),
            "/reconstruction/rgb": ("map_rgb_colored_5cm.ply", None),
            "/reconstruction/structure": ("map_building_clean_5cm.ply", None),
            "/reconstruction/ground": ("map_ground_5cm.ply", None),
            "/reconstruction/walls": ("map_walls_5cm.ply", None),
            "/reconstruction/ceiling": ("map_ceiling_5cm.ply", None),
        }
        self._entries = []
        for topic, (filename, excluded_labels) in inputs.items():
            path = scene_dir / filename
            if not path.is_file():
                raise FileNotFoundError(path)
            publisher = self.create_publisher(PointCloud2, topic, qos)
            message = make_cloud(path, frame_id, exclude_labels=excluded_labels)
            self._entries.append((topic, publisher, message))
            self.get_logger().info(f"Loaded {message.width:,} points for {topic} from {path}")
        grid_path = scene_dir / "occupancy_2d" / "occupancy_grid.npz"
        if not grid_path.is_file():
            raise FileNotFoundError(grid_path)
        grid_publisher = self.create_publisher(
            OccupancyGrid, "/reconstruction/occupancy_2d", qos
        )
        grid_message = make_occupancy_grid(grid_path, frame_id)
        self._entries.append(
            ("/reconstruction/occupancy_2d", grid_publisher, grid_message)
        )
        self.get_logger().info(
            f"Loaded {grid_message.info.width}x{grid_message.info.height} cells "
            f"for /reconstruction/occupancy_2d from {grid_path}"
        )
        self._timer = self.create_timer(5.0, self.publish_all)
        self.publish_all()

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for _, publisher, message in self._entries:
            message.header.stamp = stamp
            publisher.publish(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("--frame-id", default="world")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ScenePublisher(args.scene_dir.resolve(), args.frame_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
