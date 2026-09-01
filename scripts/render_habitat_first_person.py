#!/usr/bin/env python3
"""Render the exact Stage2 trajectory through Habitat's original RGB sensor."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import habitat_sim
import cv2
import imageio.v2 as imageio
import numpy as np
from habitat_sim.utils.common import quat_from_angle_axis


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execution", required=True, type=Path)
    parser.add_argument("--generated-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--hfov", type=float, default=90.0)
    parser.add_argument("--gpu-device-id", type=int, default=0)
    parser.add_argument("--scene-graph", type=Path)
    parser.add_argument("--candidates", type=Path)
    parser.add_argument("--show-target-boxes", action="store_true")
    parser.add_argument("--box-lead-frames", type=int, default=30)
    parser.add_argument("--box-hold-frames", type=int, default=15)
    return parser.parse_args()


def object_corners(obj: dict) -> np.ndarray:
    center = np.asarray(obj["center_xyz_m"], dtype=np.float64)
    half = 0.5 * np.asarray(obj["size_xyz_m"], dtype=np.float64)
    orientation = obj.get("orientation_wxyz", [1.0, 0.0, 0.0, 0.0])
    yaw = 2.0 * math.atan2(float(orientation[3]), float(orientation[0]))
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw), 0.0],
         [math.sin(yaw), math.cos(yaw), 0.0],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    offsets = np.asarray(
        [[sx * half[0], sy * half[1], sz * half[2]]
         for sx in (-1.0, 1.0)
         for sy in (-1.0, 1.0)
         for sz in (-1.0, 1.0)],
        dtype=np.float64,
    )
    return center + offsets @ rotation.T


def target_events(execution: dict, scene_graph: dict, candidates: dict) -> list[dict]:
    objects = {
        obj["id"]: obj
        for room in scene_graph.get("rooms", [])
        for obj in room.get("objects", [])
    }
    by_candidate = {
        candidate["id"]: candidate
        for values in candidates.get("by_task", {}).values()
        for candidate in values
    }
    events = []
    for observation in execution.get("observations", []):
        if observation.get("outcome") != "found":
            continue
        candidate = by_candidate.get(observation.get("candidate_id"))
        obj = None if candidate is None else objects.get(candidate.get("object_id"))
        if obj is None:
            continue
        label = observation.get("verification", {}).get("target_label", obj.get("label", "target"))
        events.append({
            "frame_index": int(observation.get("frame_index", 0)),
            "label": str(label).upper(),
            "corners": object_corners(obj),
        })
    return events


def overlay_target_boxes(
    rgb: np.ndarray,
    pose: np.ndarray,
    events: list[dict],
    frame_index: int,
    hfov_deg: float,
    lead_frames: int,
    hold_frames: int,
) -> np.ndarray:
    height, width = rgb.shape[:2]
    yaw = float(pose[3])
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.asarray([math.sin(yaw), -math.cos(yaw), 0.0])
    up = np.asarray([0.0, 0.0, 1.0])
    focal = 0.5 * width / math.tan(math.radians(hfov_deg) * 0.5)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for event in events:
        if not event["frame_index"] - lead_frames <= frame_index <= event["frame_index"] + hold_frames:
            continue
        relative = event["corners"] - pose[:3]
        depth = relative @ forward
        if np.count_nonzero(depth > 0.05) < 4:
            continue
        valid = depth > 0.05
        x_values = width * 0.5 + (relative[valid] @ right) * focal / depth[valid]
        y_values = height * 0.5 - (relative[valid] @ up) * focal / depth[valid]
        x0 = int(np.clip(math.floor(float(x_values.min())) - 5, 0, width - 1))
        y0 = int(np.clip(math.floor(float(y_values.min())) - 5, 0, height - 1))
        x1 = int(np.clip(math.ceil(float(x_values.max())) + 5, 0, width - 1))
        y1 = int(np.clip(math.ceil(float(y_values.max())) + 5, 0, height - 1))
        if x1 - x0 < 4 or y1 - y0 < 4 or x0 == width - 1 or y0 == height - 1:
            continue
        color = (45, 220, 70)
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (15, 15, 15), 5, cv2.LINE_AA)
        cv2.rectangle(bgr, (x0, y0), (x1, y1), color, 2, cv2.LINE_AA)
        text = f"{event['label']}  FOUND"
        (text_width, text_height), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
        )
        text_y = max(text_height + 8, y0 - 6)
        cv2.rectangle(
            bgr,
            (x0, text_y - text_height - 7),
            (min(width - 1, x0 + text_width + 8), text_y + 3),
            (15, 15, 15),
            -1,
        )
        cv2.putText(
            bgr, text, (x0 + 4, text_y - 2), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, color, 2, cv2.LINE_AA,
        )
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def main() -> None:
    args = parse_args()
    execution = json.loads(args.execution.read_text(encoding="utf-8"))
    generated = json.loads(args.generated_config.read_text(encoding="utf-8"))
    trajectory = np.asarray(execution["trajectory_xyz_yaw"], dtype=np.float64)
    initial_agent_h = np.asarray(generated["initial_agent_habitat_xyz"], dtype=np.float64)
    events = []
    if args.show_target_boxes:
        if args.scene_graph is None or args.candidates is None:
            raise ValueError("--show-target-boxes requires --scene-graph and --candidates")
        scene_graph = json.loads(args.scene_graph.read_text(encoding="utf-8"))
        candidates = json.loads(args.candidates.read_text(encoding="utf-8"))
        events = target_events(execution, scene_graph, candidates)

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(Path(generated["scene"]).resolve())
    sim_cfg.scene_dataset_config_file = str(Path(generated["scene_config"]).resolve())
    sim_cfg.enable_physics = True
    sim_cfg.gpu_device_id = args.gpu_device_id

    sensor = habitat_sim.CameraSensorSpec()
    sensor.uuid = "rgb"
    sensor.sensor_type = habitat_sim.SensorType.COLOR
    sensor.resolution = [args.height, args.width]
    sensor.position = [0.0, 1.0, 0.0]
    sensor.hfov = args.hfov
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        args.output,
        fps=args.fps,
        codec="libx264",
        quality=8,
        macro_block_size=1,
        ffmpeg_log_level="error",
        output_params=["-movflags", "+faststart"],
    )
    try:
        with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
            agent = sim.initialize_agent(0)
            for index, pose in enumerate(trajectory):
                sensor_delta_f = pose[:3] - np.asarray([0.0, 0.0, 1.0])
                state = agent.get_state()
                state.position = initial_agent_h + S_HABITAT_TO_FALCON.T @ sensor_delta_f
                state.rotation = quat_from_angle_axis(
                    float(pose[3]), np.asarray([0.0, 1.0, 0.0])
                )
                agent.set_state(state)
                rgba = np.asarray(sim.get_sensor_observations()["rgb"])
                rgb = np.ascontiguousarray(rgba[..., :3].astype(np.uint8))
                if events:
                    rgb = overlay_target_boxes(
                        rgb, pose, events, index, args.hfov,
                        args.box_lead_frames, args.box_hold_frames,
                    )
                writer.append_data(rgb)
                if index == 0 or (index + 1) % max(1, len(trajectory) // 10) == 0:
                    print(f"rendered {index + 1}/{len(trajectory)}", flush=True)
    finally:
        writer.close()
    print(f"video={args.output}")


if __name__ == "__main__":
    main()
