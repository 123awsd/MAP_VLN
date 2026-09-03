#!/usr/bin/env python3
"""Render four local third-person RGB panels for semantic viewpoint families."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import habitat_sim
import magnum as mn
import numpy as np
from PIL import Image, ImageDraw


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
FALCON_INITIAL_POSITION = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
COLORS = {
    "support_surface": (239, 132, 42),
    "surrounding_region": (139, 92, 246),
    "below_region": (42, 176, 118),
    "instance_region": (39, 145, 238),
}
TASK_ORDER = (
    "check_microwave_kitchen_l2",
    "check_coffee_table_living_l2",
    "check_bag_under_bed_l1",
    "check_tv_living_l3",
)
CALIBRATED_RGB_BOXES = {
    "L2_boxer_32": (398, 378, 508, 462),
    "L2_boxer_58": (278, 366, 552, 605),
    "L1_boxer_6": (220, 280, 741, 648),
    "L3_boxer_25": (322, 399, 581, 592),
}
BOX_EDGES = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7),
             (7, 4), (0, 4), (1, 5), (2, 6), (3, 7))


def falcon_to_habitat(points, generated):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    # Candidate poses and scene-graph boxes use the FALCON planning frame,
    # whose [0, 0, 1] origin is anchored at Habitat's initial agent position.
    initial = np.asarray(generated["initial_agent_habitat_xyz"], dtype=np.float64)
    return initial + (S_HABITAT_TO_FALCON.T @ (points - FALCON_INITIAL_POSITION).T).T


def project(points, camera, target, width, height, hfov):
    forward = target - camera
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray([0.0, 1.0, 0.0]))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    relative = points - camera
    depth = relative @ forward
    focal = width * 0.5 / math.tan(math.radians(hfov) * 0.5)
    pixels = np.column_stack((
        width * 0.5 + (relative @ right) * focal / np.maximum(depth, 1e-6),
        height * 0.5 - (relative @ up) * focal / np.maximum(depth, 1e-6),
    ))
    return pixels, depth


def box_corners(obj):
    center = np.asarray(obj["center_xyz_m"], float)
    size = np.asarray(obj["size_xyz_m"], float)
    w, x, y, z = map(float, obj.get("orientation_wxyz", [1, 0, 0, 0]))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    rotation = np.asarray([[math.cos(yaw), -math.sin(yaw), 0.0],
                           [math.sin(yaw), math.cos(yaw), 0.0],
                           [0.0, 0.0, 1.0]])
    return np.asarray([center + rotation @ (0.5 * size * [sx, sy, sz])
                       for sx, sy, sz in ((-1,-1,-1),(-1,-1,1),(-1,1,1),(-1,1,-1),
                                          (1,-1,-1),(1,-1,1),(1,1,1),(1,1,-1))])


def frustum(pose, depth=.24, hfov=48.0):
    origin = np.asarray([pose[k] for k in ("x", "y", "z")], float)
    yaw = float(pose["yaw"])
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    left = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0])
    up = np.asarray([0.0, 0.0, 1.0])
    half_w = depth * math.tan(math.radians(hfov) * .5)
    half_h = half_w * .60
    center = origin + depth * forward
    corners = [center + s * half_w * left + v * half_h * up
               for s, v in ((1,1),(-1,1),(-1,-1),(1,-1))]
    return np.vstack((origin, corners))


def diverse(candidates, maximum=8):
    if len(candidates) <= maximum:
        return candidates
    ordered = sorted(candidates, key=lambda c: float(c.get("terminal_cost", 0.0)))
    chosen = [ordered[0]]
    while len(chosen) < maximum:
        remaining = [c for c in ordered if c not in chosen]
        chosen.append(max(remaining, key=lambda c: min(
            abs(math.atan2(math.sin(float(c["candidate_angle_rad"]) - float(old["candidate_angle_rad"])),
                           math.cos(float(c["candidate_angle_rad"]) - float(old["candidate_angle_rad"]))))
            for old in chosen)))
    return chosen


def overlay(rgb, candidates, target_object, generated, camera, target, hfov):
    height, width = rgb.shape[:2]
    region_type = candidates[0]["region_type"]
    color_rgb = COLORS[region_type]
    color = color_rgb[::-1]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    layer = bgr.copy()

    x0, y0, x1, y1 = CALIBRATED_RGB_BOXES[target_object["id"]]
    scale_x, scale_y = width / 900.0, height / 650.0
    p0 = (int(round(x0 * scale_x)), int(round(y0 * scale_y)))
    p1 = (int(round(x1 * scale_x)), int(round(y1 * scale_y)))
    cv2.rectangle(layer, p0, p1, (250, 250, 250), 5, cv2.LINE_AA)
    cv2.rectangle(layer, p0, p1, color, 2, cv2.LINE_AA)

    for index, candidate in enumerate(candidates):
        pose = candidate["pose"]
        yaw = float(pose["yaw"])
        origin_f = np.asarray([pose[k] for k in ("x", "y", "z")], float)
        forward_f = origin_f + .30 * np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
        points_h = falcon_to_habitat([origin_f, forward_f], generated)
        fp, fd = project(points_h, camera, target, width, height, hfov)
        if np.any(fd <= .05):
            continue
        origin = fp[0]
        direction = fp[1] - fp[0]
        direction /= max(np.linalg.norm(direction), 1e-6)
        side = np.asarray([-direction[1], direction[0]])
        tip = origin + 34.0 * direction
        left = tip + 13.0 * side
        right = tip - 13.0 * side
        back_left = origin - 6.0 * direction + 7.0 * side
        back_right = origin - 6.0 * direction - 7.0 * side
        segments = ((back_left, back_right), (back_left, origin), (back_right, origin),
                    (origin, left), (origin, right), (left, right))
        candidate_color = (35, 198, 255) if index == 0 else color
        thickness = 2 if index == 0 else 1
        for start, end in segments:
            pa, pb = tuple(np.rint(start).astype(int)), tuple(np.rint(end).astype(int))
            cv2.line(layer, pa, pb, (250, 250, 250), thickness + 3, cv2.LINE_AA)
            cv2.line(layer, pa, pb, candidate_color, thickness, cv2.LINE_AA)
    composed = cv2.addWeighted(layer, .94, bgr, .06, 0.0)
    return cv2.cvtColor(composed, cv2.COLOR_BGR2RGB)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--generated-config", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=900)
    parser.add_argument("--height", type=int, default=650)
    parser.add_argument("--hfov", type=float, default=72.0)
    args = parser.parse_args()
    pool = json.loads(args.candidates.read_text())["by_task"]
    generated = json.loads(args.generated_config.read_text())
    graph = json.loads(args.scene_graph.read_text())
    objects = {obj["id"]: obj for obj in graph["objects"]}
    args.output_dir.mkdir(parents=True, exist_ok=True)

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = str(Path(generated["scene"]).resolve())
    sim_cfg.scene_dataset_config_file = str(Path(generated["scene_config"]).resolve())
    sim_cfg.gpu_device_id = 0
    sensor = habitat_sim.CameraSensorSpec()
    sensor.uuid = "gallery_rgb"
    sensor.sensor_type = habitat_sim.SensorType.COLOR
    sensor.resolution = [args.height, args.width]
    sensor.hfov = args.hfov
    sensor.clear_color = mn.Color4(1.0, 1.0, 1.0, 1.0)
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor]

    outputs = []
    manifest = []
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
        for panel_index, task_id in enumerate(TASK_ORDER):
            candidates = diverse(pool[task_id])
            region_f = np.asarray(candidates[0]["search_region_center_xyz_m"], float)
            target_object = objects[candidates[0]["object_id"]]
            object_f = np.asarray(target_object["center_xyz_m"], float)
            framing_target_f = object_f
            candidate_f = np.asarray([[c["pose"][k] for k in ("x","y","z")] for c in candidates])
            # A valid candidate provides a collision-free local camera anchor.
            anchor = candidate_f[0]
            camera_f = anchor.copy()
            away = anchor[:2] - framing_target_f[:2]
            away /= max(np.linalg.norm(away), 1e-6)
            camera_f[:2] += 1.00 * away
            camera_f[2] = anchor[2] + 0.75
            camera_h = falcon_to_habitat([camera_f], generated)[0]
            target_h = falcon_to_habitat([framing_target_f], generated)[0]
            node = sim.agents[0].scene_node
            node.translation = mn.Vector3(*camera_h)
            node.rotation = mn.Quaternion.from_matrix(mn.Matrix4.look_at(
                mn.Vector3(*camera_h), mn.Vector3(*target_h), mn.Vector3(0.0, 1.0, 0.0)
            ).rotation())
            rgba = np.asarray(sim.get_sensor_observations()["gallery_rgb"])
            rgb = overlay(np.ascontiguousarray(rgba[..., :3]), candidates, target_object, generated,
                          camera_h, target_h, args.hfov)
            path = args.output_dir / f"{panel_index + 1}_{candidates[0]['region_type']}.png"
            Image.fromarray(rgb).save(path)
            outputs.append(Image.fromarray(rgb))
            manifest.append({"task_id": task_id, "region_type": candidates[0]["region_type"],
                             "candidate_count_total": len(pool[task_id]),
                             "candidate_count_shown": len(candidates), "image": str(path.resolve())})

    gap = 20
    gallery = Image.new("RGB", (args.width * 2 + gap, args.height * 2 + gap), "white")
    for i, panel in enumerate(outputs):
        gallery.paste(panel, ((i % 2) * (args.width + gap), (i // 2) * (args.height + gap)))
    gallery_path = args.output_dir / "candidate_region_gallery_2x2.png"
    gallery.save(gallery_path)
    (args.output_dir / "manifest.json").write_text(json.dumps({
        "format": "pre_map_vln.candidate_region_gallery.v1",
        "candidates": str(args.candidates.resolve()), "panels": manifest,
        "gallery": str(gallery_path.resolve()),
        "legend": "yellow=selected candidate; family color=remaining valid candidates",
    }, indent=2) + "\n")
    print(gallery_path)


if __name__ == "__main__":
    main()
