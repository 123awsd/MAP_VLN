#!/usr/bin/env python3
"""Render clean, data-faithful paper assets for the 00337 case study.

The script deliberately produces source panels rather than a system-overview
composition.  Every panel has a pure-white background, no coordinate axes, and
uses only Stage-1/Stage-2 data already stored in the repository outputs.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon as Polygon2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection


ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / "outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829"
PCD = RUN / "bag_export/map_occupied.pcd"
FLOORS_JSON = ROOT / "outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix/floors.json"
BOX_ROOT = ROOT / "outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix"
ROOM_ROOT = ROOT / "outputs/room_pipeline/00337_transition_aware_qwen_v1"
CANDIDATES = ROOT / "outputs/stage2_3d/tasks/00337_representative_planning_v1_20260830/execution/candidates_with_recovery.json"
EXECUTION = ROOT / "outputs/stage2_3d/tasks/00337_representative_planning_v1_20260830/execution/habitat_execution.json"
TRANSITION_PCD = RUN / "transition_reconstruction/transition_points.pcd"
SHORT_EXECUTION = ROOT / "outputs/stage2_3d/tasks/00337_all_found_showcase_v1_20260830/execution/habitat_execution.json"
CANDIDATE_OUT = ROOT / "paper/system_overview/assets/candidates/clean_real_data"
SELECTED_OUT = ROOT / "paper/system_overview/assets/selected/clean_real_data"
FINAL_OUT = ROOT / "paper/system_overview/assets/final"
OUT = CANDIDATE_OUT

CURATED_STEMS = (
    "stage1_multifloor_categorical",
    "stage1_multifloor_structural_cutaway",
    "stage1_floor_triptych_structural",
    "stage1_multifloor_structural_exploded",
    "stage1_floor_triptych_isometric",
    "stage1_floor_triptych_open_cutaway",
    "stage1_multifloor_exploded",
    "stage1_pointcloud_with_3d_boxes",
    "floor2_pointcloud_with_selected_3d_boxes",
    "rooms_with_object_box_footprints",
    "semantic_rooms_and_3d_boxes_stack",
    "multiview_candidate_frustums_local_geometry",
    "stage2_multifloor_execution_with_observation_frustums",
)

# High-separation print-safe floor palette: dark purple → saturated blue →
# vivid green.  The luminance and hue gaps remain visible in dense point
# clouds and grayscale-ish paper previews.
FLOOR_COLORS = {1: "#440154", 2: "#225EA8", 3: "#00A65A"}
FLOOR_COLORS_ALT = {1: "#455A9C", 2: "#8B5A9F", 3: "#D66B73"}
ROOM_COLORS = {
    "bedroom": "#B8D8F0",
    "bathroom": "#A9DDD6",
    "living_room": "#F4C6A6",
    "kitchen": "#F4D98B",
    "corridor": "#D9D9D9",
    "entry_hall": "#D7C7E8",
    "unknown": "#ECECEC",
    "transition_space": "#F29E9E",
}
BOX_GROUP_COLORS = {
    "sleep": "#7B61A8",
    "seating": "#D95F76",
    "table": "#E5933A",
    "fixture": "#2A9D8F",
    "appliance": "#3274A1",
    "boundary": "#7C8796",
    "other": "#5C677D",
}


def load_ascii_pcd(path: Path) -> np.ndarray:
    header = 0
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            header += 1
            if line.strip().lower().startswith("data"):
                break
    return np.loadtxt(path, dtype=np.float32, skiprows=header, usecols=(0, 1, 2))


def load_boxes(floor: int) -> list[dict]:
    path = BOX_ROOT / f"L{floor}_boxes.csv"
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    numeric = {
        "tx_world_object", "ty_world_object", "tz_world_object", "qw_world_object",
        "qx_world_object", "qy_world_object", "qz_world_object", "scale_x",
        "scale_y", "scale_z", "prob",
    }
    for row in rows:
        for key in numeric:
            row[key] = float(row[key])
    return rows


def load_rooms(floor: int) -> list[dict]:
    data = json.loads((ROOM_ROOT / f"L{floor}_scene_graph_qwen.json").read_text())
    return data["rooms"]


def floor_bounds() -> tuple[list[dict], list[float]]:
    floors = json.loads(FLOORS_JSON.read_text())
    z = [f["floor_z_m"] for f in floors]
    bounds = [z[0] - 0.35, (z[0] + z[1]) / 2, (z[1] + z[2]) / 2, z[2] + 2.85]
    return floors, bounds


def floor_masks(points: np.ndarray) -> dict[int, np.ndarray]:
    _, b = floor_bounds()
    return {i + 1: (points[:, 2] >= b[i]) & (points[:, 2] < b[i + 1]) for i in range(3)}


def sample(points: np.ndarray, maximum: int, seed: int = 7) -> np.ndarray:
    if len(points) <= maximum:
        return points
    rng = np.random.default_rng(seed)
    return points[rng.choice(len(points), maximum, replace=False)]


def new_3d(figsize=(7.0, 7.0)):
    fig = plt.figure(figsize=figsize, facecolor="white")
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.set_axis_off()
    ax.grid(False)
    ax.set_proj_type("ortho")
    return fig, ax


def fit_3d(ax, points: np.ndarray, z_scale: float = 0.9, pad: float = 0.03):
    lo = np.nanmin(points, axis=0)
    hi = np.nanmax(points, axis=0)
    span = np.maximum(hi - lo, 1e-3)
    lo -= span * pad
    hi += span * pad
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect((span[0], span[1], span[2] * z_scale))


def save(fig, stem: str):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{stem}.png", dpi=360, bbox_inches="tight", pad_inches=0.02,
                facecolor="white", transparent=False)
    fig.savefig(OUT / f"{stem}.pdf", bbox_inches="tight", pad_inches=0.02,
                facecolor="white", transparent=False)
    plt.close(fig)


def add_floor_legend(fig):
    for x, floor in zip((.39, .50, .61), (1, 2, 3)):
        fig.text(x, .925, f"●  {floor}F", color=FLOOR_COLORS[floor],
                 fontsize=10.5, fontweight="bold", ha="center", va="center")


def quaternion_yaw(row: dict) -> float:
    w = row["qw_world_object"]
    x = row["qx_world_object"]
    y = row["qy_world_object"]
    z = row["qz_world_object"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def box_group(label: str) -> str:
    label = label.lower()
    if any(k in label for k in ("bed", "crib", "pillow", "mattress")):
        return "sleep"
    if any(k in label for k in ("sofa", "chair", "stool", "bench")):
        return "seating"
    if any(k in label for k in ("table", "desk", "countertop", "island", "nightstand")):
        return "table"
    if any(k in label for k in ("sink", "toilet", "bathtub", "vanity", "shower")):
        return "fixture"
    if any(k in label for k in ("tv", "television", "microwave", "fridge", "oven", "dishwasher")):
        return "appliance"
    if any(k in label for k in ("door", "window", "curtain")):
        return "boundary"
    return "other"


def obb_vertices(row: dict) -> np.ndarray:
    sx, sy, sz = row["scale_x"], row["scale_y"], row["scale_z"]
    local = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
    ], dtype=float) * 0.5
    yaw = quaternion_yaw(row)
    rot = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    center = np.array([row["tx_world_object"], row["ty_world_object"], row["tz_world_object"]])
    return local @ rot.T + center


BOX_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
BOX_FACES = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
             (2, 3, 7, 6), (3, 0, 4, 7)]


def draw_box_3d(ax, row: dict, color: str, alpha=0.12, linewidth=0.65, zorder=5):
    verts = obb_vertices(row)
    faces = [[verts[i] for i in face] for face in BOX_FACES]
    ax.add_collection3d(Poly3DCollection(faces, facecolors=color, edgecolors="none",
                                          alpha=alpha, zorder=zorder))
    lines = [[verts[a], verts[b]] for a, b in BOX_EDGES]
    ax.add_collection3d(Line3DCollection(lines, colors=color, linewidths=linewidth,
                                          alpha=min(1.0, alpha + 0.55), zorder=zorder + 1))


def render_multifloor(points: np.ndarray, colors: dict[int, str], stem: str):
    fig, ax = new_3d((7.0, 7.2))
    masks = floor_masks(points)
    shown = []
    for floor in (1, 2, 3):
        pts = sample(points[masks[floor]], 70000, 20 + floor)
        shown.append(pts)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.42, c=colors[floor],
                   alpha=0.72, depthshade=False, linewidths=0, rasterized=True)
    shown = np.vstack(shown)
    fit_3d(ax, shown)
    ax.view_init(elev=24, azim=-58, roll=0)
    save(fig, stem)


def render_floor_triptych(points: np.ndarray):
    fig = plt.figure(figsize=(12.0, 4.1), facecolor="white")
    masks = floor_masks(points)
    for idx, floor in enumerate((1, 2, 3), 1):
        ax = fig.add_subplot(1, 3, idx, projection="3d")
        ax.set_axis_off(); ax.set_proj_type("ortho"); ax.set_facecolor("white")
        pts = sample(points[masks[floor]], 60000, 40 + floor)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.55, c=FLOOR_COLORS[floor],
                   alpha=0.76, depthshade=False, linewidths=0, rasterized=True)
        fit_3d(ax, pts, z_scale=1.15, pad=0.01)
        ax.view_init(elev=28, azim=-58)
        ax.text2D(0.05, 0.91, f"L{floor}", transform=ax.transAxes, color=FLOOR_COLORS[floor],
                  fontsize=15, fontweight="bold")
    fig.subplots_adjust(0, 0, 1, 1, wspace=-0.06)
    save(fig, "stage1_floor_triptych_isometric")


def render_open_floor_triptych(points: np.ndarray):
    fig = plt.figure(figsize=(12.0, 4.1), facecolor="white")
    masks = floor_masks(points)
    open_mask = architectural_open_mask(points)
    for idx, floor in enumerate((1, 2, 3), 1):
        ax = fig.add_subplot(1, 3, idx, projection="3d")
        ax.set_axis_off(); ax.set_proj_type("ortho"); ax.set_facecolor("white")
        pts = sample(points[masks[floor] & open_mask], 60000, 50 + floor)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.72, c=FLOOR_COLORS[floor],
                   alpha=0.78, depthshade=False, linewidths=0, rasterized=True)
        fit_3d(ax, pts, z_scale=1.30, pad=0.01)
        ax.view_init(elev=31, azim=-61)
        ax.text2D(0.05, 0.91, f"L{floor}", transform=ax.transAxes, color=FLOOR_COLORS[floor],
                  fontsize=15, fontweight="bold")
    fig.subplots_adjust(0, 0, 1, 1, wspace=-0.07)
    save(fig, "stage1_floor_triptych_open_cutaway")


def render_exploded(points: np.ndarray):
    fig, ax = new_3d((7.1, 7.5))
    masks = floor_masks(points)
    offsets = {1: 0.0, 2: 2.15, 3: 4.30}
    all_pts = []
    for floor in (1, 2, 3):
        pts = sample(points[masks[floor]], 65000, 60 + floor).copy()
        pts[:, 2] += offsets[floor]
        all_pts.append(pts)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.48, c=FLOOR_COLORS[floor],
                   alpha=0.72, depthshade=False, linewidths=0, rasterized=True)
    all_pts = np.vstack(all_pts)
    fit_3d(ax, all_pts, z_scale=0.84)
    ax.view_init(elev=24, azim=-58)
    save(fig, "stage1_multifloor_exploded")


def architectural_open_mask(points: np.ndarray) -> np.ndarray:
    """Remove ceiling slabs and two front exterior faces for a dollhouse view."""
    floors, bounds = floor_bounds()
    keep = np.zeros(len(points), dtype=bool)
    x_front = np.quantile(points[:, 0], 0.79)
    y_front = np.quantile(points[:, 1], 0.20)
    for idx, info in enumerate(floors):
        floor = idx + 1
        fm = (points[:, 2] >= bounds[idx]) & (points[:, 2] < bounds[idx + 1])
        ceiling = points[:, 2] > float(info["floor_z_m"]) + 2.34
        front_shell = (points[:, 0] > x_front) | (points[:, 1] < y_front)
        keep |= fm & ~ceiling & ~front_shell
    return keep


def slab_free_open_mask(points: np.ndarray) -> np.ndarray:
    """Keep observed interior structure while removing floor/ceiling slabs.

    The thresholds are relative to the online-estimated floor elevations, not
    scene-specific truth geometry.  Two front shells are also opened to avoid
    hiding the interior behind exterior wall samples.
    """
    floors, bounds = floor_bounds()
    keep = np.zeros(len(points), dtype=bool)
    x_front = np.quantile(points[:, 0], 0.80)
    y_front = np.quantile(points[:, 1], 0.19)
    front_shell = (points[:, 0] > x_front) | (points[:, 1] < y_front)
    for idx, info in enumerate(floors):
        z0 = float(info["floor_z_m"])
        fm = (points[:, 2] >= bounds[idx]) & (points[:, 2] < bounds[idx + 1])
        # Remove the thick occupied bands caused by floor and ceiling returns.
        interior_z = (points[:, 2] > z0 + .34) & (points[:, 2] < z0 + 2.34)
        keep |= fm & interior_z & ~front_shell
    return keep


def vertical_structure_mask(points: np.ndarray, xy_resolution=.16,
                            minimum_span=.72, minimum_count=6) -> np.ndarray:
    """Highlight wall-like vertical columns without semantic labels."""
    if len(points) == 0:
        return np.zeros(0, dtype=bool)
    cells = np.floor(points[:, :2] / xy_resolution).astype(np.int32)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    n = int(inverse.max()) + 1
    z_min = np.full(n, np.inf)
    z_max = np.full(n, -np.inf)
    np.minimum.at(z_min, inverse, points[:, 2])
    np.maximum.at(z_max, inverse, points[:, 2])
    counts = np.bincount(inverse, minlength=n)
    structural_column = ((z_max - z_min) >= minimum_span) & (counts >= minimum_count)
    return structural_column[inverse]


def render_structural_cutaway(points: np.ndarray):
    """Single architectural cutaway with faint context and crisp wall columns."""
    masks = floor_masks(points)
    opened = slab_free_open_mask(points)
    fig, ax = new_3d((7.0, 7.2))
    shown = []
    for floor in (1, 2, 3):
        floor_points = points[masks[floor] & opened]
        context = sample(floor_points, 52000, 180 + floor)
        structural = floor_points[vertical_structure_mask(floor_points)]
        structural = sample(structural, 62000, 190 + floor)
        shown.extend([context, structural])
        ax.scatter(context[:, 0], context[:, 1], context[:, 2], s=.24,
                   c=FLOOR_COLORS[floor], alpha=.10, depthshade=False,
                   linewidths=0, rasterized=True)
        ax.scatter(structural[:, 0], structural[:, 1], structural[:, 2], s=.68,
                   c=FLOOR_COLORS[floor], alpha=.76, depthshade=False,
                   linewidths=0, rasterized=True)
    extent = np.vstack(shown)
    fit_3d(ax, extent, z_scale=1.02, pad=.015)
    ax.view_init(elev=27, azim=-58)
    add_floor_legend(fig)
    save(fig, "stage1_multifloor_structural_cutaway")


def render_structural_triptych(points: np.ndarray):
    """Per-floor views at a shared camera angle for unoccluded inspection."""
    masks = floor_masks(points)
    opened = slab_free_open_mask(points)
    fig = plt.figure(figsize=(12.0, 4.1), facecolor="white")
    for idx, floor in enumerate((1, 2, 3), 1):
        ax = fig.add_subplot(1, 3, idx, projection="3d")
        ax.set_axis_off(); ax.set_proj_type("ortho"); ax.set_facecolor("white")
        floor_points = points[masks[floor] & opened]
        context = sample(floor_points, 42000, 210 + floor)
        structural = floor_points[vertical_structure_mask(floor_points)]
        structural = sample(structural, 52000, 220 + floor)
        ax.scatter(context[:, 0], context[:, 1], context[:, 2], s=.28,
                   c=FLOOR_COLORS[floor], alpha=.10, depthshade=False,
                   linewidths=0, rasterized=True)
        ax.scatter(structural[:, 0], structural[:, 1], structural[:, 2], s=.72,
                   c=FLOOR_COLORS[floor], alpha=.80, depthshade=False,
                   linewidths=0, rasterized=True)
        fit_3d(ax, floor_points, z_scale=1.38, pad=.012)
        ax.view_init(elev=30, azim=-60)
        ax.text2D(.055, .91, f"{floor}F", transform=ax.transAxes,
                  color=FLOOR_COLORS[floor], fontsize=15, fontweight="bold")
    fig.subplots_adjust(0, 0, 1, 1, wspace=-.045)
    save(fig, "stage1_floor_triptych_structural")


def render_structural_exploded(points: np.ndarray):
    """Slightly separate floors while preserving their shared XY alignment."""
    masks = floor_masks(points)
    opened = slab_free_open_mask(points)
    offsets = {1: 0.0, 2: 1.25, 3: 2.50}
    fig, ax = new_3d((7.0, 7.4))
    shown = []
    for floor in (1, 2, 3):
        floor_points = points[masks[floor] & opened]
        structural = floor_points[vertical_structure_mask(floor_points)]
        structural = sample(structural, 65000, 240 + floor).copy()
        structural[:, 2] += offsets[floor]
        shown.append(structural)
        ax.scatter(structural[:, 0], structural[:, 1], structural[:, 2], s=.66,
                   c=FLOOR_COLORS[floor], alpha=.78, depthshade=False,
                   linewidths=0, rasterized=True)
    extent = np.vstack(shown)
    fit_3d(ax, extent, z_scale=.88, pad=.02)
    ax.view_init(elev=25, azim=-58)
    add_floor_legend(fig)
    save(fig, "stage1_multifloor_structural_exploded")


def render_cutaway(points: np.ndarray, colors: dict[int, str], stem: str, azim=-58):
    masks = floor_masks(points)
    cut = architectural_open_mask(points)
    fig, ax = new_3d((7.0, 7.2))
    shown = []
    for floor in (1, 2, 3):
        pts = sample(points[masks[floor] & cut], 65000, 80 + floor)
        shown.append(pts)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.62, c=colors[floor],
                   alpha=0.79, depthshade=False, linewidths=0, rasterized=True)
    shown = np.vstack(shown)
    fit_3d(ax, shown)
    ax.view_init(elev=27, azim=azim)
    save(fig, stem)


def render_cloud_boxes(points: np.ndarray):
    fig, ax = new_3d((7.3, 7.3))
    masks = floor_masks(points)
    active = (masks[1] | masks[2] | masks[3]) & architectural_open_mask(points)
    cloud = sample(points[active], 135000, 101)
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=0.42, c="#8494A7",
               alpha=0.43, depthshade=False, linewidths=0, rasterized=True)
    # Keep substantial, semantically useful boxes; omit doors/windows to avoid a wireframe cage.
    for floor in (1, 2, 3):
        for row in load_boxes(floor):
            group = box_group(row["name"])
            volume = row["scale_x"] * row["scale_y"] * row["scale_z"]
            if group in {"boundary", "other"} or volume < 0.025 or row["prob"] < 0.53:
                continue
            draw_box_3d(ax, row, BOX_GROUP_COLORS[group], alpha=0.14, linewidth=0.72)
    fit_3d(ax, cloud)
    ax.view_init(elev=25, azim=-58)
    save(fig, "stage1_pointcloud_with_3d_boxes")


def render_floor2_box_detail(points: np.ndarray):
    _, bounds = floor_bounds()
    fm = (points[:, 2] >= bounds[1]) & (points[:, 2] < bounds[2])
    cloud = points[fm]
    # Focus on the kitchen/living half where several meaningful instances coexist.
    crop = (cloud[:, 0] > -0.8) & (cloud[:, 0] < 4.8) & (cloud[:, 1] > 8.0) & (cloud[:, 1] < 16.6)
    cloud = sample(cloud[crop], 75000, 117)
    fig, ax = new_3d((6.8, 6.2))
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=0.58, c="#9AA8B8",
               alpha=0.42, depthshade=False, linewidths=0, rasterized=True)
    selected_labels = {"microwave", "countertop", "sofa", "coffee table", "television"}
    for row in load_boxes(2):
        x, y = row["tx_world_object"], row["ty_world_object"]
        if not (-0.8 < x < 4.8 and 8.0 < y < 16.6) or row["prob"] < 0.54:
            continue
        group = box_group(row["name"])
        if group == "boundary" or row["name"].lower() not in selected_labels:
            continue
        draw_box_3d(ax, row, BOX_GROUP_COLORS[group], alpha=0.17, linewidth=0.82)
    fit_3d(ax, cloud, z_scale=1.25, pad=0.02)
    ax.view_init(elev=31, azim=-63)
    save(fig, "floor2_pointcloud_with_selected_3d_boxes")


def render_short_execution(points: np.ndarray):
    data = json.loads(SHORT_EXECUTION.read_text())
    traj = np.asarray(data["trajectory_xyz_yaw"], dtype=float)
    if traj.ndim != 2 or traj.shape[1] < 3:
        return
    xyz = traj[:, :3]
    # Keep the real continuous trajectory but remove exact repeats for a cleaner vector line.
    moving = np.r_[True, np.linalg.norm(np.diff(xyz, axis=0), axis=1) > 1e-4]
    xyz = xyz[moving]
    masks = floor_masks(points)
    cloud = sample(points[masks[1] | masks[2] | masks[3]], 90000, 151)
    fig, ax = new_3d((7.2, 7.0))
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=0.32, c="#AEB8C4",
               alpha=0.12, depthshade=False, linewidths=0, rasterized=True)
    _, bounds = floor_bounds()
    segs, cols = [], []
    for a, b in zip(xyz[:-1], xyz[1:]):
        z = (a[2] + b[2]) / 2
        floor = 1 if z < bounds[1] else 2 if z < bounds[2] else 3
        segs.append(np.vstack([a, b])); cols.append(FLOOR_COLORS[floor])
    ax.add_collection3d(Line3DCollection(segs, colors=cols, linewidths=1.75, alpha=0.95))
    observations = data.get("observations", [])
    for idx, obs in enumerate(observations):
        pose = obs.get("pose")
        if not pose or len(pose) < 4:
            continue
        p = {"x": pose[0], "y": pose[1], "z": pose[2], "yaw": pose[3]}
        outcome = obs.get("verification_outcome", obs.get("outcome", ""))
        color = "#2A9D8F" if outcome == "found" else "#D1495B"
        ax.add_collection3d(Line3DCollection(frustum_segments(p, 90, 70, 0.42),
                                              colors=color, linewidths=0.72, alpha=0.75))
    fit_3d(ax, cloud)
    ax.view_init(elev=26, azim=-58)
    save(fig, "stage2_multifloor_execution_with_observation_frustums")


def render_transition_highlight(points: np.ndarray):
    transitions = load_ascii_pcd(TRANSITION_PCD)
    masks = floor_masks(points)
    cloud = sample(points[masks[1] | masks[2] | masks[3]], 110000, 123)
    fig, ax = new_3d((7.0, 7.0))
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=0.34, c="#AEB8C4",
               alpha=0.17, depthshade=False, linewidths=0, rasterized=True)
    ax.scatter(transitions[:, 0], transitions[:, 1], transitions[:, 2], s=1.8,
               c=transitions[:, 2], cmap="autumn", alpha=0.92, depthshade=False,
               linewidths=0, rasterized=True)
    fit_3d(ax, cloud)
    ax.view_init(elev=25, azim=-58)
    save(fig, "stage1_cross_floor_transition_spaces")


def rectangle_xy(row: dict) -> np.ndarray:
    sx, sy = row["scale_x"] / 2, row["scale_y"] / 2
    corners = np.array([[-sx, -sy], [sx, -sy], [sx, sy], [-sx, sy]])
    yaw = quaternion_yaw(row)
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return corners @ rot.T + np.array([row["tx_world_object"], row["ty_world_object"]])


def render_rooms_boxes():
    fig, axes = plt.subplots(1, 3, figsize=(12.2, 4.15), facecolor="white")
    for ax, floor in zip(axes, (1, 2, 3)):
        ax.set_aspect("equal"); ax.axis("off"); ax.set_facecolor("white")
        rooms = load_rooms(floor)
        all_xy = []
        for room in rooms:
            poly = np.asarray(room["polygon_xy_m"], dtype=float)
            if len(poly) < 3:
                continue
            all_xy.append(poly)
            room_type = room.get("qwen_semantic_type") or room.get("space_role", "unknown")
            color = ROOM_COLORS.get(room_type, ROOM_COLORS["unknown"])
            ax.add_patch(Polygon2D(poly, closed=True, facecolor=color, edgecolor="#FFFFFF",
                                   linewidth=1.6, alpha=0.88, joinstyle="round"))
            c = np.asarray(room["centroid_xy_m"])
            if room_type != "transition_space":
                ax.text(c[0], c[1], room_type.replace("_", "\n"), ha="center", va="center",
                        fontsize=6.4, color="#314254", fontweight="medium")
        boxes = load_boxes(floor)
        for row in boxes:
            group = box_group(row["name"])
            if group == "boundary" or row["prob"] < 0.55:
                continue
            poly = rectangle_xy(row)
            ax.add_patch(Polygon2D(poly, closed=True, facecolor="none",
                                   edgecolor=BOX_GROUP_COLORS[group], linewidth=0.65, alpha=0.78))
        xy = np.vstack(all_xy)
        margin = 0.35
        ax.set_xlim(xy[:, 0].min() - margin, xy[:, 0].max() + margin)
        ax.set_ylim(xy[:, 1].min() - margin, xy[:, 1].max() + margin)
        ax.text(0.03, 0.96, f"L{floor}", transform=ax.transAxes, va="top", ha="left",
                color=FLOOR_COLORS[floor], fontsize=14, fontweight="bold")
    fig.subplots_adjust(0.005, 0.005, 0.995, 0.995, wspace=0.03)
    save(fig, "rooms_with_object_box_footprints")


def render_semantic_stack():
    floors, _ = floor_bounds()
    fig, ax = new_3d((7.4, 7.0))
    all_xyz = []
    for floor_info in floors:
        floor = int(floor_info["floor"])
        z = float(floor_info["floor_z_m"]) + 0.08
        for room in load_rooms(floor):
            poly = np.asarray(room["polygon_xy_m"], dtype=float)
            if len(poly) < 3:
                continue
            xyz = np.column_stack([poly, np.full(len(poly), z)])
            all_xyz.append(xyz)
            room_type = room.get("qwen_semantic_type") or room.get("space_role", "unknown")
            color = ROOM_COLORS.get(room_type, ROOM_COLORS["unknown"])
            ax.add_collection3d(Poly3DCollection([xyz], facecolors=color, edgecolors="white",
                                                  linewidths=0.7, alpha=0.90))
        for row in load_boxes(floor):
            group = box_group(row["name"])
            if group == "boundary" or row["prob"] < 0.58:
                continue
            draw_box_3d(ax, row, BOX_GROUP_COLORS[group], alpha=0.13, linewidth=0.5)
    all_xyz = np.vstack(all_xyz)
    fit_3d(ax, all_xyz, z_scale=0.86, pad=0.04)
    ax.view_init(elev=32, azim=-58)
    save(fig, "semantic_rooms_and_3d_boxes_stack")


def frustum_segments(pose: dict, hfov_deg: float, vfov_deg: float, length: float) -> list[np.ndarray]:
    origin = np.array([pose["x"], pose["y"], pose["z"]], dtype=float)
    yaw = float(pose["yaw"])
    hf = math.radians(hfov_deg) / 2
    vf = math.radians(vfov_deg) / 2
    corners = []
    for h, v in [(-hf, -vf), (hf, -vf), (hf, vf), (-hf, vf)]:
        horizontal = length * math.cos(v)
        corners.append(origin + np.array([
            horizontal * math.cos(yaw + h), horizontal * math.sin(yaw + h), length * math.sin(v)
        ]))
    segs = [np.vstack([origin, c]) for c in corners]
    for i in range(4):
        segs.append(np.vstack([corners[i], corners[(i + 1) % 4]]))
    return segs


def render_candidate_frustums(points: np.ndarray):
    data = json.loads(CANDIDATES.read_text())["by_task"]
    task = "check_microwave_kitchen_l2"
    candidates = data[task]
    executed = json.loads(EXECUTION.read_text())
    selected_ids = {o["candidate_id"] for o in executed.get("observations", []) if o["task_id"] == task}
    center = np.asarray(candidates[0]["search_region_center_xyz_m"], dtype=float)
    local = points[(np.abs(points[:, 0] - center[0]) < 3.1) &
                   (np.abs(points[:, 1] - center[1]) < 3.1) &
                   (np.abs(points[:, 2] - center[2]) < 2.0)]
    local = sample(local, 50000, 131)
    fig, ax = new_3d((7.0, 6.4))
    ax.scatter(local[:, 0], local[:, 1], local[:, 2], s=0.65, c="#BAC3CD",
               alpha=0.24, depthshade=False, linewidths=0, rasterized=True)
    # Search surface as a compact translucent region.
    size = np.asarray(candidates[0]["search_region_size_xyz_m"], dtype=float)
    synthetic = {
        "tx_world_object": center[0], "ty_world_object": center[1], "tz_world_object": center[2],
        "qw_world_object": 1.0, "qx_world_object": 0.0, "qy_world_object": 0.0,
        "qz_world_object": 0.0, "scale_x": size[0], "scale_y": size[1], "scale_z": max(size[2], 0.08),
    }
    draw_box_3d(ax, synthetic, "#F39C34", alpha=0.25, linewidth=1.0, zorder=8)
    for idx, cand in enumerate(candidates):
        chosen = cand["id"] in selected_ids or idx == 0
        color = "#D1495B" if chosen else "#277DA1"
        width = 1.55 if chosen else 0.75
        alpha = 0.95 if chosen else 0.50
        segments = frustum_segments(cand["pose"], cand.get("horizontal_fov_deg", 90),
                                    cand.get("vertical_fov_deg", 70), 0.58)
        ax.add_collection3d(Line3DCollection(segments, colors=color, linewidths=width, alpha=alpha))
        p = cand["pose"]
        ax.scatter([p["x"]], [p["y"]], [p["z"]], s=18 if chosen else 9,
                   c=color, edgecolors="white", linewidths=0.4, depthshade=False)
    extent = np.vstack([local, center[None, :]])
    fit_3d(ax, extent, z_scale=1.15, pad=0.04)
    ax.view_init(elev=31, azim=-57)
    save(fig, "multiview_candidate_frustums_local_geometry")


def main():
    points = load_ascii_pcd(PCD)
    masks = floor_masks(points)
    points = points[masks[1] | masks[2] | masks[3]]
    render_multifloor(points, FLOOR_COLORS, "stage1_multifloor_categorical")
    render_multifloor(points, FLOOR_COLORS_ALT, "stage1_multifloor_categorical_alt")
    render_structural_cutaway(points)
    render_structural_triptych(points)
    render_structural_exploded(points)
    render_floor_triptych(points)
    render_open_floor_triptych(points)
    render_exploded(points)
    render_cutaway(points, FLOOR_COLORS, "stage1_cutaway_corner_categorical")
    render_cutaway(points, FLOOR_COLORS_ALT, "stage1_cutaway_corner_alt", azim=-48)
    render_cloud_boxes(points)
    render_floor2_box_detail(points)
    render_transition_highlight(points)
    render_rooms_boxes()
    render_semantic_stack()
    render_candidate_frustums(points)
    render_short_execution(points)
    SELECTED_OUT.mkdir(parents=True, exist_ok=True)
    for stem in CURATED_STEMS:
        for suffix in (".png", ".pdf"):
            shutil.copy2(CANDIDATE_OUT / f"{stem}{suffix}", SELECTED_OUT / f"{stem}{suffix}")
    FINAL_OUT.mkdir(parents=True, exist_ok=True)
    for suffix in (".png", ".pdf"):
        shutil.copy2(CANDIDATE_OUT / f"stage1_multifloor_structural_exploded{suffix}",
                     FINAL_OUT / f"stage1_mapping_structure{suffix}")
        shutil.copy2(CANDIDATE_OUT / f"stage1_floor_triptych_structural{suffix}",
                     FINAL_OUT / f"stage1_mapping_floors{suffix}")
    print(f"generated candidates in {CANDIDATE_OUT}")
    print(f"copied reviewed assets to {SELECTED_OUT}")


if __name__ == "__main__":
    main()
