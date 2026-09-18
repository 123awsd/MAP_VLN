#!/usr/bin/env python3
"""Render clean floor-perimeter tubes as real 3D scene geometry."""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import habitat_sim
import magnum as mn
import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull

from render_stage2_textured_trajectory import (
    BACKGROUND_COLORS,
    completion_events_in_habitat,
    draw_xray_trajectory,
    trajectory_in_habitat,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--execution", required=True, type=Path)
    parser.add_argument("--generated-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=2160)
    parser.add_argument("--frame-index", type=int, default=780)
    parser.add_argument("--orbit-frame-count", type=int, default=929)
    parser.add_argument("--hfov", type=float, default=72.0)
    parser.add_argument("--elevation-deg", type=float, default=24.0)
    parser.add_argument("--orbit-padding", type=float, default=0.86)
    parser.add_argument("--line-radius", type=float, default=0.035)
    return parser.parse_args()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def glb_positions_in_habitat(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    json_length, json_type = struct.unpack_from("<II", raw, 12)
    if json_type != 0x4E4F534A:
        raise ValueError("Invalid GLB JSON chunk")
    document = json.loads(raw[20 : 20 + json_length].rstrip(b" \x00"))
    bin_header = 20 + json_length
    bin_length, bin_type = struct.unpack_from("<II", raw, bin_header)
    if bin_type != 0x004E4942:
        raise ValueError("Invalid GLB BIN chunk")
    binary = raw[bin_header + 8 : bin_header + 8 + bin_length]
    arrays = []
    seen = set()
    for mesh in document.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            accessor_index = primitive.get("attributes", {}).get("POSITION")
            if accessor_index is None or accessor_index in seen:
                continue
            seen.add(accessor_index)
            accessor = document["accessors"][accessor_index]
            view = document["bufferViews"][accessor["bufferView"]]
            offset = int(view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
            stride = int(view.get("byteStride", 12))
            count = int(accessor["count"])
            if stride == 12:
                xyz = np.frombuffer(binary, "<f4", count=count * 3, offset=offset).reshape(-1, 3)
            else:
                xyz = np.asarray(
                    [np.frombuffer(binary, "<f4", count=3, offset=offset + i * stride) for i in range(count)]
                )
            # HM3D GLB (x, y, z) maps to Habitat (x, z, -y).
            arrays.append(np.column_stack((xyz[:, 0], xyz[:, 2], -xyz[:, 1])))
    return np.concatenate(arrays, axis=0)


def floor_perimeters(scene_path: Path) -> list[np.ndarray]:
    vertices = glb_positions_in_habitat(scene_path)
    bands = ((-2.82, -0.05, -2.68), (-0.05, 2.72, 0.12), (2.72, 8.20, 2.92))
    perimeters = []
    for low, high, line_y in bands:
        horizontal = vertices[(vertices[:, 1] >= low) & (vertices[:, 1] < high)][:, (0, 2)]
        # Quantization removes redundant scan vertices before the hull operation.
        horizontal = np.unique(np.round(horizontal, 3), axis=0)
        hull = ConvexHull(horizontal)
        ring_xz = horizontal[hull.vertices]
        ring = np.column_stack(
            (ring_xz[:, 0], np.full(len(ring_xz), line_y), ring_xz[:, 1])
        )
        perimeters.append(np.vstack((ring, ring[0])))
    return perimeters


def main() -> None:
    args = parse_args()
    execution = load_json(args.execution.resolve())
    generated = load_json(args.generated_config.resolve())
    points = trajectory_in_habitat(execution, generated)
    events = completion_events_in_habitat(execution, generated, 1.0, 90.0)
    completion_points = np.asarray(
        [event["point"] for event in events if np.linalg.norm(event["point"] - points[-1]) > 0.10],
        dtype=np.float64,
    ).reshape(-1, 3)
    perimeters = floor_perimeters(args.scene.resolve())

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(args.scene.resolve())
    sim_cfg.enable_physics = False
    sensor_spec = habitat_sim.CameraSensorSpec()
    sensor_spec.uuid = "orbit_rgb"
    sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    sensor_spec.resolution = [args.height, args.width]
    sensor_spec.position = mn.Vector3(0.0, 0.0, 0.0)
    sensor_spec.hfov = args.hfov
    sensor_spec.clear_color = BACKGROUND_COLORS["light-gray"]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_spec]

    colors = (
        mn.Color4(1.00, 0.72, 0.30, 1.0),
        mn.Color4(0.20, 1.00, 0.62, 1.0),
        mn.Color4(0.82, 0.48, 1.00, 1.0),
    )
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        for index, (ring, color) in enumerate(zip(perimeters, colors), start=1):
            sim.add_trajectory_object(
                f"floor_{index}_perimeter",
                [mn.Vector3(*point) for point in ring],
                num_segments=16,
                radius=args.line_radius,
                color=color,
                smooth=False,
            )
        scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
        target = np.asarray(scene_bb.center(), dtype=np.float64)
        scene_size = np.asarray(scene_bb.size(), dtype=np.float64)
        vfov = 2.0 * math.atan(math.tan(math.radians(args.hfov) * 0.5) * args.height / args.width)
        distance = args.orbit_padding * 0.5 * float(np.linalg.norm(scene_size)) / math.sin(vfov * 0.5)
        elevation = math.radians(args.elevation_deg)
        angle = 2.0 * math.pi * args.frame_index / args.orbit_frame_count
        camera_position = target + np.asarray(
            [distance * math.cos(elevation) * math.cos(angle), distance * math.sin(elevation),
             distance * math.cos(elevation) * math.sin(angle)], dtype=np.float64
        )
        camera_mn, target_mn = mn.Vector3(*camera_position), mn.Vector3(*target)
        node = sim.agents[0].scene_node
        node.translation = camera_mn
        node.rotation = mn.Quaternion.from_matrix(
            mn.Matrix4.look_at(camera_mn, target_mn, mn.Vector3(0.0, 1.0, 0.0)).rotation()
        )
        rgba = np.asarray(sim.get_sensor_observations()["orbit_rgb"])
        rgb = np.ascontiguousarray(rgba[..., :3].astype(np.uint8))

    # Keep exactly the established trajectory proportions; only floor lines
    # are new 3D geometry.
    scale = max(1.0, args.width / 1920.0)
    rgb = draw_xray_trajectory(
        rgb, points, camera_position, target, args.hfov,
        max(2, round(3 * scale)), max(4, round(6 * scale)), max(0, round(10 * scale)),
        completion_points, max(4, round(6 * scale)), max(4, round(6 * scale)), [],
        max(2, round(3 * scale)), (float(points[:, 1].min()), float(points[:, 1].max())), True,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb).save(args.output, compress_level=2)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
