#!/usr/bin/env python3
"""Render a publication-style Boxer detection-to-fusion story for any episode.

The four panels use only recorded RGB/depth poses and Boxer CSV products:
2D detections, frame-local 3D OBBs, local raw-to-fused evidence, and the global
fused object map.  No scene-specific labels or geometry are hard-coded.
"""

from __future__ import annotations

import argparse
import colorsys
import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


BOX_EDGES = (
    (0, 1), (1, 3), (3, 2), (2, 0),
    (4, 5), (5, 7), (7, 6), (6, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)
PALETTE = (
    "#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00",
    "#56B4E9", "#7A5195", "#EF5675", "#2F4B7C", "#43AA8B",
)
VIVID_PALETTE = (
    "#0066FF", "#FF3B30", "#00A86B", "#AF52DE", "#FF9500",
    "#00B8D9", "#FF2D95", "#6B8E23", "#7C3AED", "#E63946",
)
ACTIVE_PALETTE = PALETTE
LINE_SCALE = 1.0
USE_VIVID = False
COLOR_OVERRIDES: dict[str, str] = {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--boxer-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frame-id", type=int, default=24)
    parser.add_argument("--prefix", default="indoor_v2")
    parser.add_argument("--local-radius", type=float, default=5.5)
    parser.add_argument("--max-points", type=int, default=90000)
    parser.add_argument("--min-prob", type=float, default=0.18)
    parser.add_argument("--compact", action="store_true", help="Use a tighter, paper-ready canvas")
    parser.add_argument("--no-panel-titles", action="store_true")
    parser.add_argument("--no-footer", action="store_true")
    parser.add_argument("--omit-local-fusion", action="store_true")
    parser.add_argument("--vivid-boxes", action="store_true")
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def label_color(label: str) -> str:
    if label in COLOR_OVERRIDES:
        return COLOR_OVERRIDES[label]
    index = int(hashlib.sha1(label.encode("utf-8")).hexdigest()[:8], 16)
    if USE_VIVID:
        hue = (index % 997) / 997.0
        red, green, blue = colorsys.hsv_to_rgb(hue, 0.88, 0.92)
        return "#{:02X}{:02X}{:02X}".format(round(red * 255), round(green * 255), round(blue * 255))
    return ACTIVE_PALETTE[index % len(ACTIVE_PALETTE)]


def quat_matrix_wxyz(q: np.ndarray) -> np.ndarray:
    q = q.astype(float)
    q /= max(np.linalg.norm(q), 1e-12)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def obb_corners(row: dict[str, str]) -> np.ndarray:
    center = np.array([f(row, "tx_world_object"), f(row, "ty_world_object"), f(row, "tz_world_object")])
    scale = np.array([f(row, "scale_x"), f(row, "scale_y"), f(row, "scale_z")])
    signs = np.array([
        [-1, -1, -1], [1, -1, -1], [-1, 1, -1], [1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [-1, 1, 1], [1, 1, 1],
    ], dtype=float)
    rotation = quat_matrix_wxyz(np.array([
        f(row, "qw_world_object"), f(row, "qx_world_object"),
        f(row, "qy_world_object"), f(row, "qz_world_object"),
    ]))
    return center + (rotation @ (signs * scale / 2.0).T).T


def read_pcd_xyz(path: Path) -> np.ndarray:
    data_line = None
    with path.open("rb") as handle:
        for index, line in enumerate(handle):
            if line.strip().lower().startswith(b"data"):
                if b"ascii" not in line.lower():
                    raise ValueError(f"only ASCII PCD is supported: {path}")
                data_line = index + 1
                break
    if data_line is None:
        raise ValueError(f"invalid PCD header: {path}")
    points = np.loadtxt(path, skiprows=data_line, usecols=(0, 1, 2), dtype=np.float32)
    return points


def uniform_sample(points: np.ndarray, limit: int) -> np.ndarray:
    if len(points) <= limit:
        return points
    return points[np.linspace(0, len(points) - 1, limit, dtype=int)]


def draw_image_boxes(ax, image: np.ndarray, rows: list[dict[str, str]], title: str, detector_size: tuple[int, int]) -> None:
    ax.imshow(image)
    sx = image.shape[1] / detector_size[0]
    sy = image.shape[0] / detector_size[1]
    ordered = sorted(rows, key=lambda row: f(row, "prob"), reverse=True)
    for row in ordered:
        x1, y1 = f(row, "x1") * sx, f(row, "y1") * sy
        x2, y2 = f(row, "x2") * sx, f(row, "y2") * sy
        color = label_color(row["name"])
        ax.add_patch(Rectangle((x1, y1), x2-x1, y2-y1, fill=False, lw=1.7 * LINE_SCALE, ec=color))
        label_x = float(np.clip(x1, 3, image.shape[1] - 80))
        label_y = float(np.clip(y1 - 3, 3, image.shape[0] - 8))
        ax.text(label_x, label_y, f'{row["name"]} {f(row, "prob"):.2f}', color="white",
                fontsize=6.5, va="bottom", ha="left",
                bbox=dict(facecolor=color, edgecolor="none", alpha=0.90, pad=1.2), clip_on=True)
    ax.set_title(title, loc="left", fontsize=11, fontweight="semibold", pad=7)
    ax.axis("off")


def project_world(points: np.ndarray, position: np.ndarray, orientation_xyzw: np.ndarray,
                  intrinsics: tuple[float, float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    # Episode orientation describes camera optical axes in the Falcon world.
    x, y, z, w = orientation_xyzw
    world_from_camera = quat_matrix_wxyz(np.array([w, x, y, z]))
    camera = (world_from_camera.T @ (points - position).T).T
    valid = camera[:, 2] > 0.04
    fx, fy, cx, cy = intrinsics
    uv = np.empty((len(points), 2), dtype=float)
    uv[:, 0] = fx * camera[:, 0] / np.maximum(camera[:, 2], 1e-9) + cx
    uv[:, 1] = fy * camera[:, 1] / np.maximum(camera[:, 2], 1e-9) + cy
    return uv, valid


def draw_projected_obbs(ax, image: np.ndarray, rows: list[dict[str, str]], position: np.ndarray,
                        orientation: np.ndarray, intrinsics: tuple[float, float, float, float],
                        title: str = "B  Per-frame 3D lifting") -> int:
    ax.imshow(image)
    shown = 0
    for row in rows:
        corners = obb_corners(row)
        uv, valid = project_world(corners, position, orientation, intrinsics)
        if valid.sum() < 4:
            continue
        color = label_color(row["name"])
        edge_count = 0
        for a, b in BOX_EDGES:
            if valid[a] and valid[b]:
                ax.plot(uv[[a, b], 0], uv[[a, b], 1], color=color, lw=1.65 * LINE_SCALE, alpha=0.98)
                edge_count += 1
        if edge_count:
            visible_uv = uv[valid]
            anchor = visible_uv[np.argmin(visible_uv[:, 1])]
            label_x = float(np.clip(anchor[0], 3, image.shape[1] - 65))
            label_y = float(np.clip(anchor[1], 9, image.shape[0] - 5))
            ax.text(label_x, label_y, row["name"], color="white", fontsize=6.5,
                    bbox=dict(facecolor=color, edgecolor="none", alpha=0.9, pad=1.1), clip_on=True)
            shown += 1
    ax.set_xlim(0, image.shape[1])
    ax.set_ylim(image.shape[0], 0)
    if title:
        ax.set_title(title, loc="left", fontsize=11, fontweight="semibold", pad=7)
    ax.axis("off")
    return shown


def draw_obb_3d(ax, row: dict[str, str], color: str, lw: float, alpha: float) -> None:
    corners = obb_corners(row)
    for a, b in BOX_EDGES:
        ax.plot(corners[[a, b], 0], corners[[a, b], 1], corners[[a, b], 2],
                color=color, lw=lw, alpha=alpha, solid_capstyle="round")


def style_3d(ax, title: str, points: np.ndarray, elev: float = 31, azim: float = -58) -> None:
    if title:
        ax.set_title(title, loc="left", fontsize=11, fontweight="semibold", pad=7)
    ax.set_facecolor("white")
    ax.grid(False)
    ax.set_axis_off()
    ax.view_init(elev=elev, azim=azim)
    if len(points):
        mins, maxs = points.min(axis=0), points.max(axis=0)
        center = (mins + maxs) / 2
        span = np.maximum(maxs - mins, 0.2)
        ax.set_xlim(center[0] - span[0]/2, center[0] + span[0]/2)
        ax.set_ylim(center[1] - span[1]/2, center[1] + span[1]/2)
        ax.set_zlim(mins[2] - 0.1, maxs[2] + 0.1)
        ax.set_box_aspect((span[0], span[1], max(span[2], 0.35 * max(span[0], span[1]))))


def main() -> None:
    global ACTIVE_PALETTE, LINE_SCALE, USE_VIVID, COLOR_OVERRIDES
    args = parse_args()
    if args.vivid_boxes:
        ACTIVE_PALETTE = VIVID_PALETTE
        LINE_SCALE = 1.32
        USE_VIVID = True
    episode_dir = args.run_dir / "episode"
    manifest = json.loads((episode_dir / "manifest.json").read_text(encoding="utf-8"))
    frame_paths = sorted(episode_dir.glob("frame_*.npz"))
    if not 0 <= args.frame_id < len(frame_paths):
        raise SystemExit(f"frame-id {args.frame_id} outside [0, {len(frame_paths)-1}]")
    with np.load(frame_paths[args.frame_id]) as frame:
        image = frame["rgb"].copy()
        position = frame["position"].astype(float)
        orientation = frame["orientation_xyzw"].astype(float)
        time_ns = int(frame["time_ns"])

    detections = read_csv(args.boxer_dir / "owl_2dbbs.csv")
    raw = read_csv(args.boxer_dir / f"{args.prefix}_3dbbs.csv")
    fused = read_csv(args.boxer_dir / f"{args.prefix}_3dbbs_fused.csv")
    all_frame_det = [row for row in detections if int(row["frame_id"]) == args.frame_id]
    selected_indices = [index for index, row in enumerate(all_frame_det) if f(row, "prob") >= args.min_prob]
    frame_det = [all_frame_det[index] for index in selected_indices]
    # Match by frame timestamp; tolerate legacy exports whose integer precision changed.
    timestamps = sorted({int(row["time_ns"]) for row in raw})
    nearest_time = min(timestamps, key=lambda value: abs(value - time_ns))
    all_frame_raw = [row for row in raw if int(row["time_ns"]) == nearest_time]
    if len(all_frame_raw) != len(all_frame_det):
        raise RuntimeError(
            f"2D/3D row mismatch at frame {args.frame_id}: "
            f"{len(all_frame_det)} detections versus {len(all_frame_raw)} lifted OBBs"
        )
    # Boxer writes 2D detections and their lifted 3D OBBs in the same order.
    # Apply the 2D display threshold once and preserve that one-to-one pairing.
    frame_raw = [all_frame_raw[index] for index in selected_indices]
    active_classes = {row["name"] for row in frame_det}
    if args.vivid_boxes:
        # Guarantee strong separation among the classes highlighted in A/B;
        # the same colors carry through to their instances in the global map.
        ordered_active = list(dict.fromkeys(row["name"] for row in frame_det))
        COLOR_OVERRIDES = {
            label: VIVID_PALETTE[index % len(VIVID_PALETTE)]
            for index, label in enumerate(ordered_active)
        }

    points = read_pcd_xyz(args.run_dir / "bag_export" / "map_occupied.pcd")
    horizontal = np.linalg.norm(points[:, :2] - position[:2], axis=1)
    local_points = points[(horizontal <= args.local_radius) & (np.abs(points[:, 2] - position[2]) <= 2.2)]
    local_points = uniform_sample(local_points, args.max_points)
    # Panel C tells one clean fusion story: this frame's raw OBB evidence and
    # nearby fused instances of the same open-vocabulary classes.
    local_raw = frame_raw
    local_fused = [row for row in fused if row["name"] in active_classes and
                   np.linalg.norm(np.array([f(row, "tx_world_object"), f(row, "ty_world_object")]) - position[:2]) <= args.local_radius]

    floor_points = points[(points[:, 2] >= position[2] - 1.25) & (points[:, 2] <= position[2] + 1.65)]
    floor_points = uniform_sample(floor_points, args.max_points)
    floor_fused = [row for row in fused if position[2] - 1.3 <= f(row, "tz_world_object") <= position[2] + 1.7]

    figure_width = 15.2 if args.omit_local_fusion else 20
    fig = plt.figure(figsize=(figure_width, 4.15 if args.compact else 5.6), dpi=220, facecolor="white")
    widths = (1.05, 1.05, 1.32) if args.omit_local_fusion else (1.05, 1.05, 1.0, 1.25)
    grid = fig.add_gridspec(1, len(widths), width_ratios=widths, wspace=0.018)
    ax_a = fig.add_subplot(grid[0, 0])
    ax_b = fig.add_subplot(grid[0, 1])
    if args.omit_local_fusion:
        ax_c = None
        ax_d = fig.add_subplot(grid[0, 2], projection="3d")
    else:
        ax_c = fig.add_subplot(grid[0, 2], projection="3d")
        ax_d = fig.add_subplot(grid[0, 3], projection="3d")

    detector_size = (int(frame_det[0]["img_width"]), int(frame_det[0]["img_height"])) if frame_det else (960, 960)
    panel_a_title = "" if args.no_panel_titles else "A  Open-vocabulary 2D detection"
    panel_b_title = "" if args.no_panel_titles else "B  Per-frame 3D lifting"
    panel_c_title = "" if args.no_panel_titles else "C  Local fusion\n     raw evidence → fused OBB"
    panel_d_title = "" if args.no_panel_titles else "D  Fused open-vocabulary object map"
    draw_image_boxes(ax_a, image, frame_det, panel_a_title, detector_size)
    projected = draw_projected_obbs(ax_b, image, frame_raw, position, orientation,
                                    (manifest["fx"], manifest["fy"], manifest["cx"], manifest["cy"]),
                                    panel_b_title)

    if ax_c is not None:
        if len(local_points):
            ax_c.scatter(local_points[:, 0], local_points[:, 1], local_points[:, 2], s=0.28,
                         c="#4F5963", alpha=0.28, linewidths=0, rasterized=True)
        for row in local_raw:
            draw_obb_3d(ax_c, row, "#9AA1A8", 0.70, 0.45)
        for row in local_fused:
            draw_obb_3d(ax_c, row, label_color(row["name"]), 1.55 * LINE_SCALE, 0.96)
        ax_c.scatter(*position, marker="^", s=35, c="#111111", depthshade=False)
        style_3d(ax_c, panel_c_title, local_points)
        ax_c.text2D(0.02, 0.02, "raw", color="#7D848B", fontsize=7, transform=ax_c.transAxes)
        ax_c.text2D(0.13, 0.02, "fused", color="#0066FF", fontsize=7, fontweight="bold", transform=ax_c.transAxes)

    if len(floor_points):
        ax_d.scatter(floor_points[:, 0], floor_points[:, 1], floor_points[:, 2], s=0.24,
                     c="#59616A", alpha=0.27 if args.vivid_boxes else 0.38,
                     linewidths=0, rasterized=True)
    for row in floor_fused:
        draw_obb_3d(ax_d, row, label_color(row["name"]), 1.15 * LINE_SCALE, 0.96)
    ax_d.scatter(*position, marker="^", s=28, c="#111111", depthshade=False)
    style_3d(ax_d, panel_d_title, floor_points, elev=34, azim=-62)

    if not args.no_footer:
        fig.text(0.012, 0.018,
                 f"Frame {args.frame_id:03d}  |  {len(frame_det)} detections  |  "
                 f"{len(frame_raw)} lifted OBBs ({projected} projected)  |  {len(fused)} fused instances",
                 fontsize=8, color="#59636D")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, bbox_inches="tight", pad_inches=0.08, facecolor="white")
    plt.close(fig)
    metadata = {
        "frame_id": args.frame_id,
        "frame_file": frame_paths[args.frame_id].name,
        "frame_time_ns": time_ns,
        "matched_box_time_ns": nearest_time,
        "detections": len(frame_det),
        "frame_raw_obbs": len(frame_raw),
        "projected_obbs": projected,
        "local_raw_obbs": len(local_raw),
        "local_fused_obbs": len(local_fused),
        "floor_fused_obbs": len(floor_fused),
        "all_fused_obbs": len(fused),
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"generated={args.output}")
    print(json.dumps(metadata))


if __name__ == "__main__":
    main()
