#!/usr/bin/env python3
"""Overlay the executed Stage2 L2 route and task boxes on the official RGB top view."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


COLORS = {
    "microwave": (230, 126, 34),
    "toilet paper": (36, 158, 125),
    "coffee table": (126, 87, 170),
    "toward L3": (42, 120, 181),
}

# Pixel-accurate annotations on the official L2 RGB orthographic plate.
# The plate is a presentation render rather than a metric raster, so these
# were calibrated against the terminal RGB observations instead of using the
# metric scene-graph boxes directly.
OFFICIAL_L2_BOXES = {
    "microwave": (410, 474, 460, 532),
    "toilet paper": (830, 501, 849, 533),
    "coffee table": (126, 457, 184, 524),
}


def world_to_pixel(xy, extent, pixel_bounds):
    """Official top view: image u=-Falcon Y, image v=-Falcon X."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    xmin, xmax, ymin, ymax = extent
    u0, u1, v0, v1 = pixel_bounds
    u = u0 + (ymax - xy[:, 1]) / (ymax - ymin) * (u1 - u0)
    v = v0 + (xmax - xy[:, 0]) / (xmax - xmin) * (v1 - v0)
    return np.column_stack((u, v))


def task_box_corners(obj):
    center = np.asarray(obj["center_xyz_m"][:2], dtype=float)
    # Tiny objects such as the toilet-paper box collapse under a paper-scale
    # outline. Preserve its center/orientation but enforce a 0.36 m minimum
    # display footprint so every task annotation remains an empty rectangle.
    half = 0.5 * np.maximum(np.asarray(obj["size_xyz_m"][:2], dtype=float), 0.36)
    w, _, _, z = map(float, obj["orientation_wxyz"])
    yaw = 2.0 * math.atan2(z, w)
    local = np.asarray([[-half[0], -half[1]], [half[0], -half[1]],
                        [half[0], half[1]], [-half[0], half[1]]])
    rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]])
    return local @ rotation.T + center


def observation_frustum(pose, depth=0.62, hfov_deg=55.0):
    """Return a top-down fan-shaped camera frustum in Falcon XY coordinates."""
    x, y, _, yaw = map(float, pose)
    apex = np.asarray([x, y])
    half_angle = math.radians(hfov_deg) / 2.0
    angles = np.linspace(yaw - half_angle, yaw + half_angle, 15)
    arc = apex + depth * np.column_stack((np.cos(angles), np.sin(angles)))
    return apex, arc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topdown", type=Path, required=True)
    parser.add_argument("--execution", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--grid-metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    full = np.asarray(Image.open(args.topdown).convert("RGB"))
    # Connected foreground ranges in the official three-floor sheet. L2 is
    # the middle component; retain its original black surround.
    crop_top, crop_bottom = 1266, 2382
    image = full[crop_top:crop_bottom].copy()
    foreground = image.max(axis=2) > 25
    # Replace only the exterior connected black canvas with paper-friendly
    # white; retain dark furniture and interior details.
    dark = (image.max(axis=2) <= 25).astype(np.uint8)
    _, components = cv2.connectedComponents(dark, connectivity=8)
    border_labels = np.unique(np.concatenate((components[0], components[-1],
                                               components[:, 0], components[:, -1])))
    exterior = np.isin(components, border_labels[border_labels != 0])
    image[exterior] = 255
    yy, xx = np.where(foreground)
    pixel_bounds = (float(xx.min()), float(xx.max()), float(yy.min()), float(yy.max()))

    metadata = json.loads(args.grid_metadata.read_text())
    origin = metadata["origin_xy_m"]
    resolution = float(metadata["resolution_m"])
    extent = (float(origin[0]), float(origin[0] + metadata["width"] * resolution),
              float(origin[1]), float(origin[1] + metadata["height"] * resolution))
    execution = json.loads(args.execution.read_text())
    trajectory = np.asarray(execution["trajectory_xyz_yaw"], dtype=float)
    floor_z = np.asarray([-0.2, 2.6, 5.4])
    floor = np.argmin(abs(trajectory[:, None, 2] - (floor_z[None, :] + 1.2)), axis=1)
    l2_indices = np.flatnonzero(floor == 1)
    first, last = int(l2_indices[0]), int(l2_indices[-1])
    route_pixels = world_to_pixel(trajectory[first:last + 1, :2], extent, pixel_bounds)

    graph = json.loads(args.scene_graph.read_text())
    wanted = ("microwave", "toilet paper", "coffee table")
    objects = {obj["label"]: obj for obj in graph["objects"]
               if int(obj.get("floor_id", 0)) == 2 and obj.get("label") in wanted}
    observation_records = {item["task_id"]: item for item in execution["observations"]}
    observations = {task_id: int(item["frame_index"])
                    for task_id, item in observation_records.items()}
    task_ids = {
        "microwave": "check_microwave_kitchen_l2",
        "toilet paper": "check_toilet_paper_bath_l2",
        "coffee table": "check_coffee_table_living_l2",
    }
    stops = [
        ("microwave", observations["check_microwave_kitchen_l2"]),
        ("toilet paper", observations["check_toilet_paper_bath_l2"]),
        ("coffee table", observations["check_coffee_table_living_l2"]),
    ]

    def draw(labeled, path):
        canvas = image.copy()
        previous = first
        for label, stop in stops:
            stop = min(max(stop, previous), last)
            pixels = world_to_pixel(trajectory[previous:stop + 1, :2], extent, pixel_bounds)
            polyline = np.rint(pixels).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [polyline], False, (255, 255, 255), 10, cv2.LINE_AA)
            cv2.polylines(canvas, [polyline], False, COLORS[label], 5, cv2.LINE_AA)
            previous = stop
        for label in wanted:
            pose = observation_records[task_ids[label]]["pose"]
            apex_xy, arc_xy = observation_frustum(pose)
            apex = np.rint(world_to_pixel([apex_xy], extent, pixel_bounds)[0]).astype(np.int32)
            arc = np.rint(world_to_pixel(arc_xy, extent, pixel_bounds)).astype(np.int32)
            fan = np.vstack((apex, arc)).reshape(-1, 1, 2)
            tint = canvas.copy()
            cv2.fillPoly(tint, [fan], COLORS[label], cv2.LINE_AA)
            canvas = cv2.addWeighted(tint, 0.14, canvas, 0.86, 0.0)
            boundary = np.vstack((apex, arc[0], arc, arc[-1], apex)).reshape(-1, 1, 2)
            cv2.polylines(canvas, [boundary], False, (255, 255, 255), 4, cv2.LINE_AA)
            cv2.polylines(canvas, [boundary], False, COLORS[label], 2, cv2.LINE_AA)
        for number, label in enumerate(wanted, start=1):
            obj = objects[label]
            x0, y0, x1, y1 = OFFICIAL_L2_BOXES[label]
            polygon = np.asarray([[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
                                 dtype=np.int32).reshape(-1, 1, 2)
            color = COLORS[label]
            cv2.polylines(canvas, [polygon], True, (255, 255, 255), 5, cv2.LINE_AA)
            cv2.polylines(canvas, [polygon], True, color, 2, cv2.LINE_AA)
            center = np.asarray([(x0 + x1) / 2.0, (y0 + y1) / 2.0])
            if labeled:
                text = f"{number}  {label}"
                anchor = tuple((np.rint(center) + np.asarray([13, -13])).astype(int))
                cv2.putText(canvas, text, anchor, cv2.FONT_HERSHEY_SIMPLEX, .58,
                            (255, 255, 255), 5, cv2.LINE_AA)
                cv2.putText(canvas, text, anchor, cv2.FONT_HERSHEY_SIMPLEX, .58,
                            color, 2, cv2.LINE_AA)
        Image.fromarray(canvas).save(path)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    draw(True, args.output)
    clean = args.output.with_name(args.output.stem + "_clean" + args.output.suffix)
    draw(False, clean)
    report = {
        "format": "pre_map_vln.stage2_l2_rgb_trajectory.v1",
        "background": str(args.topdown.resolve()), "background_role": "HM3D reference RGB top view",
        "execution": str(args.execution.resolve()), "trajectory_source": "executed Stage2 trajectory",
        "l2_pose_range_inclusive": [first, last], "l2_pose_count": last - first + 1,
        "task_boxes": [{"label": label, "id": objects[label]["id"],
                         "center_xyz_m": objects[label]["center_xyz_m"]} for label in wanted],
        "world_extent_xy_m": list(extent), "pixel_bounds_uv": list(pixel_bounds),
        "axis_mapping": "u=-FalconY, v=-FalconX", "labeled_png": str(args.output.resolve()),
        "clean_png": str(clean.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
