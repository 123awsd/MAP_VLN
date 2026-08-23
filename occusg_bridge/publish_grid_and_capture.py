#!/usr/bin/env python3
"""Publish one saved occupancy grid and capture OccuSG room polygons."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from incremental_dude_msgs.msg import Region2DArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class GridRoundTrip(Node):
    def __init__(self, grid_path: Path, metadata_path: Path, output_path: Path):
        super().__init__("pre_map_vln_grid_round_trip")
        self.grid = np.load(grid_path).astype(np.int8, copy=False)
        self.metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        self.output_path = output_path
        self.completed = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "/mapUAV", qos)
        self.subscription = self.create_subscription(
            Region2DArray, "/dude/regions", self.capture_regions, 10
        )
        self.timer = self.create_timer(0.5, self.publish_grid)

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = float(self.metadata["resolution_m"])
        msg.info.width = int(self.grid.shape[1])
        msg.info.height = int(self.grid.shape[0])
        msg.info.origin.position.x = float(self.metadata["origin_xy_m"][0])
        msg.info.origin.position.y = float(self.metadata["origin_xy_m"][1])
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.reshape(-1).astype(int).tolist()
        self.publisher.publish(msg)

    def capture_regions(self, msg: Region2DArray):
        regions = []
        for region in msg.regions:
            regions.append({
                "id": int(region.id),
                "centroid_xy_m": [float(region.centroid.x), float(region.centroid.y)],
                "area_m2": float(region.area),
                "adjacent_ids": [int(value) for value in region.adjacent_ids],
                "polygon_xy_m": [
                    [float(point.x), float(point.y)] for point in region.polygon.points
                ],
                "convex_hull_xy_m": [
                    [float(point.x), float(point.y)] for point in region.convex_hull.points
                ],
            })
        result = {
            "format": "pre_map_vln.occusg_regions.v1",
            "source_grid": str(self.metadata.get("source_episode", "")),
            "captured_at_unix_s": time.time(),
            "region_count": len(regions),
            "regions": regions,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(
            json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        self.get_logger().info(
            f"Captured {len(regions)} regions in {self.output_path}"
        )
        self.completed = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--grid", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = GridRoundTrip(args.grid, args.metadata, args.output)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.completed and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.completed:
            raise TimeoutError("OccuSG did not publish /dude/regions before timeout")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
