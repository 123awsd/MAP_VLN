#!/usr/bin/env python3
"""Render a textured paper frame with subtle, depth-aware floor color washes."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import habitat_sim
import magnum as mn
import numpy as np
from PIL import Image

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
    parser.add_argument("--boundaries-y", type=float, nargs=2, default=(-1.20, 1.55))
    parser.add_argument("--tint-alpha", type=float, default=0.18)
    parser.add_argument("--room-outlines", action="store_true")
    parser.add_argument(
        "--no-trajectory",
        action="store_true",
        help="Render the same textured/tinted scene without trajectory or task markers.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def visible_world_y(
    depth: np.ndarray,
    camera_position: np.ndarray,
    target: np.ndarray,
    hfov_deg: float,
) -> np.ndarray:
    height, width = depth.shape
    forward = target - camera_position
    forward /= np.linalg.norm(forward)
    world_up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    camera_up = np.cross(right, forward)
    focal = 0.5 * width / math.tan(math.radians(hfov_deg) * 0.5)
    rows = np.arange(height, dtype=np.float32)[:, None]
    cols = np.arange(width, dtype=np.float32)[None, :]
    vertical = -(rows - height * 0.5) / focal
    horizontal = (cols - width * 0.5) / focal
    # Habitat depth is distance along the camera ray, not only its forward
    # component. Reconstruct the normalized world ray before reading height.
    ray_y = forward[1] + horizontal * right[1] + vertical * camera_up[1]
    ray_norm = np.sqrt(1.0 + horizontal * horizontal + vertical * vertical)
    return camera_position[1] + depth * ray_y / ray_norm


def apply_floor_tint(
    rgb: np.ndarray,
    depth: np.ndarray,
    world_y: np.ndarray,
    boundaries: tuple[float, float],
    alpha: float,
) -> np.ndarray:
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("tint alpha must be in [0, 1]")
    valid = np.isfinite(depth) & (depth > 0.02) & (depth < 100.0)
    levels = np.digitize(world_y, sorted(boundaries), right=False)
    # Distinct pastel washes; low alpha preserves the original RGB texture.
    palette = np.asarray(
        [[255, 181, 112], [103, 226, 161], [190, 139, 237]], dtype=np.float32
    )
    tinted = rgb.astype(np.float32)
    for level in range(3):
        mask = valid & (levels == level)
        tinted[mask] = (1.0 - alpha) * tinted[mask] + alpha * palette[level]
    return np.clip(tinted, 0, 255).astype(np.uint8)


def add_room_outlines(rgb: np.ndarray, depth: np.ndarray) -> np.ndarray:
    """Draw only long, smoothed depth discontinuities as subtle room lines."""
    valid = np.isfinite(depth) & (depth > 0.02) & (depth < 100.0)
    work = depth.copy()
    finite = work[valid]
    work[~valid] = float(np.percentile(finite, 95)) if finite.size else 30.0
    smooth = cv2.GaussianBlur(work, (0, 0), sigmaX=2.2)
    gx = cv2.Sobel(smooth, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(smooth, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)
    threshold = max(0.08, float(np.percentile(mag[valid], 94.5)))
    edges = ((mag >= threshold) & valid).astype(np.uint8) * 255
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    scale = max(1.0, rgb.shape[1] / 1920.0)
    selected = [c for c in contours if cv2.arcLength(c, False) > 45.0 * scale]
    canvas = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.drawContours(canvas, selected, -1, (248, 248, 248), max(1, round(2 * scale)), cv2.LINE_AA)
    cv2.drawContours(canvas, selected, -1, (125, 130, 138), max(1, round(0.8 * scale)), cv2.LINE_AA)
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)


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

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(args.scene.resolve())
    sim_cfg.enable_physics = False
    sensor_specs = []
    for uuid, sensor_type in (
        ("orbit_rgb", habitat_sim.SensorType.COLOR),
        ("orbit_depth", habitat_sim.SensorType.DEPTH),
    ):
        spec = habitat_sim.CameraSensorSpec()
        spec.uuid = uuid
        spec.sensor_type = sensor_type
        spec.resolution = [args.height, args.width]
        spec.position = mn.Vector3(0.0, 0.0, 0.0)
        spec.hfov = args.hfov
        if sensor_type == habitat_sim.SensorType.COLOR:
            spec.clear_color = BACKGROUND_COLORS["light-gray"]
        sensor_specs.append(spec)
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
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
        observations = sim.get_sensor_observations()
        rgb = np.ascontiguousarray(np.asarray(observations["orbit_rgb"])[..., :3].astype(np.uint8))
        depth = np.asarray(observations["orbit_depth"], dtype=np.float32)

    world_y = visible_world_y(depth, camera_position, target, args.hfov)
    rgb = apply_floor_tint(rgb, depth, world_y, tuple(args.boundaries_y), args.tint_alpha)
    if args.room_outlines:
        rgb = add_room_outlines(rgb, depth)

    if not args.no_trajectory:
        # Preserve the established paper trajectory thickness and white halo.
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
