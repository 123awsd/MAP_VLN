#!/usr/bin/env python3
"""Render a dense, scene-centric TRO-style system overview.

The figure is intentionally built from stored experimental artifacts only:
Stage-1 point cloud and 3-D boxes, the Qwen task graph, feasible viewpoints,
executed A*/B-spline segments, Habitat RGB, and OWLv2 detections.
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
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

import render_four_task_system_overview as base


ROOT = base.ROOT
RUN = base.RUN
OUT = ROOT / "paper/system_overview/assets/candidates/tro_system_overview"
SELECTED = ROOT / "paper/system_overview/assets/selected/tro_system_overview"
FINAL = ROOT / "paper/system_overview/assets/final"
BOX_ROOT = ROOT / "outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix"
SCENE_GRAPH = RUN / "execution/updated_scene_graph.json"
SEMANTIC_MEMORY = ROOT / "paper/system_overview/assets/selected/clean_real_data/semantic_rooms_and_3d_boxes_stack.png"

INK = "#172633"
MUTED = "#667684"
HAIR = "#CBD4DA"
STRUCTURE = {1: "#456D8A", 2: "#278B78", 3: "#B76A32"}
FLOOR_FILL = {1: "#DCEAF2", 2: "#DDEFEA", 3: "#F3E5D9"}
ROOM_COLORS = {
    "kitchen": "#F1C37B", "bathroom": "#7FC8BE", "living_room": "#A994C8",
    "bedroom": "#E99A9A", "dining_room": "#91BBDD", "hallway": "#D8C67D",
    "stairwell": "#AEB8C1", "unknown": "#D8DEE2",
}
TASK = base.COLORS
FONT = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")
META_ZH = {
    base.TASKS[0]: ("微波炉", "二楼 / 厨房", "靠近台面"),
    base.TASKS[1]: ("卫生纸", "二楼 / 卫生间", "靠近马桶"),
    base.TASKS[2]: ("茶几", "二楼 / 客厅", "靠近沙发"),
    base.TASKS[3]: ("电视", "三楼 / 客厅", "实例本体"),
}


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_box_tables():
    tables = {}
    for floor in (1, 2, 3):
        path = BOX_ROOT / f"L{floor}_boxes.csv"
        with path.open("r", encoding="utf-8") as handle:
            for index, row in enumerate(csv.DictReader(handle)):
                object_id = f"L{floor}_boxer_{index}"
                center = np.asarray([float(row[f"t{axis}_world_object"]) for axis in "xyz"])
                size = np.asarray([float(row[f"scale_{axis}"]) for axis in "xyz"])
                yaw = 2 * math.atan2(float(row["qz_world_object"]), float(row["qw_world_object"]))
                tables[object_id] = dict(center=center, size=size, yaw=yaw, name=row["name"])
    return tables


def box_geometry(center, size, yaw=0.0):
    signs = np.asarray([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ], float)
    points = signs * np.asarray(size)[None, :] / 2
    c, s = math.cos(yaw), math.sin(yaw)
    points = points @ np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T + center
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
             [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return [points[list(edge)] for edge in edges], [points[face] for face in faces]


def add_box(ax, box, color, alpha=.08, lw=1.0, zorder=5):
    edges, faces = box_geometry(box["center"], box["size"], box["yaw"])
    ax.add_collection3d(Poly3DCollection(faces, facecolors=color, edgecolors="none",
                                         alpha=alpha, zorder=zorder))
    ax.add_collection3d(Line3DCollection(edges, colors=color, linewidths=lw,
                                          alpha=.96, zorder=zorder + 1))


def floor_cloud(points, z0, maximum, seed):
    mask = (points[:, 2] > z0 + .16) & (points[:, 2] < z0 + 2.34)
    cloud = base.sample(points[mask], maximum, seed)
    # An open front-right cut exposes the interior while retaining room geometry.
    x_cut = np.quantile(cloud[:, 0], .89)
    y_cut = np.quantile(cloud[:, 1], .10)
    return cloud[(cloud[:, 0] < x_cut) & (cloud[:, 1] > y_cut)]


def add_floor_slab(ax, cloud, z, color):
    lo, hi = np.quantile(cloud[:, :2], .02, axis=0), np.quantile(cloud[:, :2], .98, axis=0)
    x0, y0 = lo; x1, y1 = hi
    face = [[(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]]
    ax.add_collection3d(Poly3DCollection(face, facecolors=color, edgecolors="none", alpha=.045))


def draw_room_memory(ax, scene_graph, floor_info):
    target_rooms = {"L2_R8", "L2_R3", "L2_R12", "L3_R8"}
    for room in scene_graph["rooms"]:
        floor = int(room["floor_id"])
        if floor not in (1, 2, 3):
            continue
        z = float(floor_info[floor - 1]["floor_z_m"]) + .20
        polygon = np.asarray(room["polygon_xy_m"], float)
        vertices = [[(x, y, z) for x, y in polygon]]
        room_type = room.get("semantic_type") or "unknown"
        color = ROOM_COLORS.get(room_type, ROOM_COLORS["unknown"])
        ax.add_collection3d(Poly3DCollection(vertices, facecolors=color,
                                             edgecolors=color, linewidths=.28,
                                             alpha=.16 if room["id"] in target_rooms else .085,
                                             zorder=2))
        if room["id"] in target_rooms:
            cx, cy = room["centroid_xy_m"]
            ax.text(cx, cy, z + .05, room_type.replace("_", " "), fontsize=5.3,
                    color="#50606D", ha="center", va="center", zorder=5)


def draw_scene(ax, candidates, execution, points, floor_info, boxes, scene_graph):
    base.clean(ax)
    ax.set_proj_type("ortho")
    clouds = []
    for floor in (1, 2, 3):
        z0 = float(floor_info[floor - 1]["floor_z_m"])
        cloud = floor_cloud(points, z0, 80000, 131 + floor)
        clouds.append(cloud)
        add_floor_slab(ax, cloud, z0 + .12, FLOOR_FILL[floor])
        rel_z = np.clip((cloud[:, 2] - z0) / 2.35, 0, 1)
        base_rgb = np.asarray(plt.matplotlib.colors.to_rgb(STRUCTURE[floor]))
        colors = base_rgb[None, :] * (.72 + .28 * rel_z[:, None])
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=.38, c=colors,
                   alpha=.39, linewidths=0, depthshade=False, rasterized=True, zorder=1)
        cxy = np.median(cloud[:, :2], axis=0)
        ax.text(cxy[0] - 7.2, cxy[1] + 5.5, z0 + 1.05, f"L{floor}",
                color=STRUCTURE[floor], fontsize=9.5, fontweight="bold", zorder=15)

    draw_room_memory(ax, scene_graph, floor_info)

    selected = {o["candidate_id"]: o for o in execution["observations"]}
    # All feasible views are part of the spatial evidence, not a detached chart.
    for task_id, group in candidates.items():
        color = TASK[task_id]
        for candidate in group:
            chosen = candidate["id"] in selected
            ax.add_collection3d(Line3DCollection(
                base.frustum3d(candidate["pose"], .84 if chosen else .55),
                colors=color, linewidths=1.45 if chosen else .55,
                alpha=.98 if chosen else .34, zorder=10 if chosen else 3))

    paths = []
    for segment in execution["executed_segments"]:
        task_id = segment["task_id"]
        path = np.asarray(segment["bspline_points_xyz_m"], float)
        paths.append(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color="white", lw=4.8,
                alpha=.94, solid_capstyle="round", zorder=11)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color=TASK[task_id], lw=2.55,
                solid_capstyle="round", zorder=12)

    start = np.asarray(execution["trajectory_xyz_yaw"][0][:3])
    ax.scatter(*start[:, None], marker="o", s=42, color=INK, edgecolor="white",
               linewidth=.8, depthshade=False, zorder=15)
    ax.text(start[0], start[1], start[2] - .34, "START", fontsize=6.0, color=INK,
            ha="center", fontweight="bold", zorder=16)

    for sequence, observation in enumerate(execution["observations"], 1):
        task_id = observation["task_id"]
        candidate = next(c for c in candidates[task_id] if c["id"] == observation["candidate_id"])
        target = np.asarray(candidate["target_xyz_m"])
        object_box = boxes.get(candidate["object_id"])
        if object_box:
            add_box(ax, object_box, TASK[task_id], alpha=.13, lw=1.25, zorder=7)
        anchor_box = boxes.get(candidate.get("anchor_object_id"))
        if anchor_box:
            add_box(ax, anchor_box, "#697A87", alpha=.025, lw=.62, zorder=4)
            pair = np.vstack([anchor_box["center"], object_box["center"] if object_box else target])
            ax.plot(pair[:, 0], pair[:, 1], pair[:, 2], color=TASK[task_id], lw=.8,
                    ls=(0, (2, 2)), alpha=.72, zorder=8)
        pose = np.asarray(observation["pose"][:3])
        ax.scatter([pose[0]], [pose[1]], [pose[2]], s=58, color=TASK[task_id],
                   edgecolor="white", linewidth=1.0, depthshade=False, zorder=16)
        label, place, relation = base.META[task_id]
        ax.text(target[0], target[1], target[2] + .42, f"{sequence}", color="white",
                fontsize=7.1, ha="center", va="center", fontweight="bold", zorder=20,
                bbox=dict(boxstyle="circle,pad=.23", fc=TASK[task_id], ec="white",
                          lw=.75, alpha=.98))

    extent = np.vstack(clouds + paths)
    base.fit3d(ax, extent, z_scale=.93, pad=.015)
    ax.view_init(elev=24, azim=-59)
    ax.dist = 2.78
    return clouds


def draw_language_strip(ax, graph, locale="en"):
    base.clean(ax)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.text(.00, .93, "(a)", fontsize=10.5, color=INK, fontweight="bold", va="top")
    heading = "语言 → 结构化空间目标" if locale == "zh" else "Language → structured spatial goals"
    ax.text(.035, .93, heading, fontsize=9.0, color=INK, fontweight="bold", va="top",
            fontproperties=FONT_BOLD if locale == "zh" else None)
    instruction = graph["instruction"].replace("朋友快到了，", "").replace("茶几和三楼", "茶几和\n三楼")
    ax.text(.035, .63, instruction, fontsize=8.15, color=INK, fontproperties=FONT,
            va="top", linespacing=1.4, transform=ax.transAxes)
    items = [
        (base.TASKS[0], "microwave", "L2 / kitchen", "near(countertop)"),
        (base.TASKS[2], "coffee table", "L2 / living", "near(sofa)"),
        (base.TASKS[3], "television", "L3 / living", "instance"),
        (base.TASKS[1], "toilet paper", "L2 / bathroom", "near(toilet)"),
    ]
    if locale == "zh":
        items = [(task_id, *META_ZH[task_id]) for task_id, *_ in items]
    x0, dx = .52, .12
    for i, (task_id, target, place, relation) in enumerate(items):
        x = x0 + i * dx
        ax.plot([x, x + .105], [.76, .76], color=TASK[task_id], lw=2.6,
                solid_capstyle="round", transform=ax.transAxes)
        ax.text(x, .64, target, fontsize=7.05, color=INK, fontweight="bold",
                transform=ax.transAxes, fontproperties=FONT_BOLD if locale == "zh" else None)
        ax.text(x, .42, f"{place}\n{relation}", fontsize=6.15, color=MUTED,
                linespacing=1.28, transform=ax.transAxes,
                fontproperties=FONT if locale == "zh" else None)
    summary = ("Qwen · 4个独立目标 · 用户未指定顺序 · 由三维几何代价确定执行顺序" if locale == "zh" else
               "Qwen · 4 independent goals  ·  no prescribed order  ·  geometry chooses execution sequence")
    ax.text(.52, .12, summary, fontsize=6.55, color=MUTED, transform=ax.transAxes,
            fontproperties=FONT if locale == "zh" else None)
    ax.plot([0, 1], [.02, .02], color=HAIR, lw=.7, transform=ax.transAxes)


def draw_semantic_memory(ax, locale="en"):
    base.clean(ax)
    image = base.crop_white_image(SEMANTIC_MEMORY, pad=4)
    ax.imshow(image)
    footer = "在线房间区域 · 三维OBB语义记忆 · 1–3层" if locale == "zh" else "online room regions  ·  3-D OBB memory  ·  L1–L3"
    ax.text(.03, .025, footer, transform=ax.transAxes, fontsize=5.65, color=MUTED,
            fontproperties=FONT if locale == "zh" else None)


def local_cloud(points, candidates):
    first = candidates[base.TASKS[0]][0]
    center = np.asarray(first["search_region_center_xyz_m"])
    mask = ((np.abs(points[:, 0] - center[0]) < 2.55) &
            (np.abs(points[:, 1] - center[1]) < 2.85) &
            (points[:, 2] > 2.72) & (points[:, 2] < 4.72))
    return base.sample(points[mask], 42000, 219)


def draw_spatial_inset(ax, candidates, execution, points, boxes, locale="en"):
    base.clean(ax); ax.set_proj_type("ortho")
    task_id = base.TASKS[0]
    group = candidates[task_id]
    chosen = execution["observations"][0]["candidate_id"]
    cloud = local_cloud(points, candidates)
    rel = (cloud[:, 2] - cloud[:, 2].min()) / max(np.ptp(cloud[:, 2]), 1e-3)
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=.45,
               c=plt.get_cmap("Blues")(rel), alpha=.34, linewidth=0,
               depthshade=False, rasterized=True)
    target_box = boxes[group[0]["object_id"]]
    anchor_box = boxes[group[0]["anchor_object_id"]]
    add_box(ax, anchor_box, "#536675", alpha=.05, lw=1.0)
    add_box(ax, target_box, TASK[task_id], alpha=.18, lw=1.4)
    region = dict(center=np.asarray(group[0]["search_region_center_xyz_m"]),
                  size=np.asarray(group[0]["search_region_size_xyz_m"]), yaw=anchor_box["yaw"])
    add_box(ax, region, "#D9A037", alpha=.14, lw=1.1)
    for c in group:
        is_chosen = c["id"] == chosen
        ax.add_collection3d(Line3DCollection(base.frustum3d(c["pose"], .76 if is_chosen else .48),
                                             colors=TASK[task_id],
                                             linewidths=1.35 if is_chosen else .40,
                                             alpha=.96 if is_chosen else .20))
    extent = np.vstack([cloud, anchor_box["center"], target_box["center"]] +
                       [np.asarray([[c["pose"][k] for k in ("x", "y", "z")]]) for c in group])
    base.fit3d(ax, extent, z_scale=.95, pad=.01)
    ax.view_init(elev=22, azim=-55); ax.dist = 2.68
    ax.text(anchor_box["center"][0], anchor_box["center"][1], anchor_box["center"][2] - .17,
            "台面锚点" if locale == "zh" else "countertop anchor", fontsize=5.6,
            color="#536675", ha="center", fontproperties=FONT if locale == "zh" else None)
    ax.text(target_box["center"][0], target_box["center"][1], target_box["center"][2] + .23,
            "微波炉" if locale == "zh" else "microwave", fontsize=5.7, color=TASK[task_id],
            ha="center", fontweight="bold", fontproperties=FONT_BOLD if locale == "zh" else None)


def draw_spatial_legend(ax, candidates, execution, locale="en"):
    base.clean(ax)
    group = candidates[base.TASKS[0]]
    chosen = execution["observations"][0]["candidate_id"]
    relation = "靠近(台面)" if locale == "zh" else "near(countertop)"
    chain = "→ 周边搜索区域 → q = (x, y, z, ψ)" if locale == "zh" else "→  surrounding region  →  q = (x, y, z, ψ)"
    ax.text(0, .92, relation, color=TASK[base.TASKS[0]], fontsize=7.0,
            fontweight="bold", fontproperties=FONT_BOLD if locale == "zh" else None)
    ax.text(.31, .92, chain, color=INK, fontsize=6.5,
            fontproperties=FONT if locale == "zh" else None)
    quality = [c["view_quality"] for c in group]
    order = np.argsort([-q for q in quality])
    x = np.linspace(.03, .97, len(group))
    for rank, idx in enumerate(order):
        c = group[idx]; selected = c["id"] == chosen
        h = .25 + .31 * c["view_quality"]
        ax.plot([x[rank], x[rank]], [.11, h], color=TASK[base.TASKS[0]] if selected else "#AAB6BE",
                lw=2.0 if selected else .8)
        ax.scatter([x[rank]], [h], s=34 if selected else 13,
                   color=TASK[base.TASKS[0]] if selected else "#AAB6BE", edgecolor="white", lw=.5)
        ax.text(x[rank], h + .055, f"{len(c['visible_region_sample_ids'])}/{c['region_sample_count']}",
                ha="center", fontsize=5.2, color=TASK[base.TASKS[0]] if selected else MUTED)
    filters = "三维FREE ∩ 无碰撞 ∩ 视场内 ∩ 视线无遮挡" if locale == "zh" else "3-D FREE  ∩  collision-free  ∩  FoV  ∩  line-of-sight"
    ax.text(0, .015, filters, fontsize=5.8, color=MUTED,
            fontproperties=FONT if locale == "zh" else None)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)


def draw_execution_ribbon(ax, execution, floor_info, locale="en"):
    title = "(e) 联合优化 → 连续三维执行" if locale == "zh" else "(e) Joint optimization → continuous 3-D execution"
    ax.text(0, .92, title, fontsize=8.3, color=INK, fontweight="bold", va="top",
            transform=ax.transAxes, fontproperties=FONT_BOLD if locale == "zh" else None)
    ax.text(.73, .92, r"$J=\sum (L_{3D}+\lambda_t T+\lambda_q C_{view})$",
            fontsize=6.5, color=MUTED, va="top", ha="center", transform=ax.transAxes)
    trajectory = np.asarray(execution["trajectory_xyz_yaw"], float)
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(trajectory[:, :3], axis=0), axis=1))]
    for floor, info in enumerate(floor_info, 1):
        z0 = float(info["floor_z_m"])
        ax.axhspan(z0 + .38, z0 + 1.72, color=FLOOR_FILL[floor], alpha=.55, zorder=0)
        ax.text(distance[-1] * 1.007, z0 + 1.05, f"L{floor}", fontsize=5.3,
                color=STRUCTURE[floor], va="center", fontweight="bold")
    previous = 0
    for task_id, observation in zip(base.TASKS, execution["observations"]):
        pose = np.asarray(observation["pose"][:3], float)
        index = int(np.argmin(np.linalg.norm(trajectory[:, :3] - pose, axis=1)))
        ax.plot(distance[previous:index + 1], trajectory[previous:index + 1, 2],
                color="white", lw=3.2, solid_capstyle="round", zorder=2)
        ax.plot(distance[previous:index + 1], trajectory[previous:index + 1, 2],
                color=TASK[task_id], lw=1.8, solid_capstyle="round", zorder=3)
        ax.scatter([distance[index]], [trajectory[index, 2]], s=27, color=TASK[task_id],
                   edgecolor="white", lw=.65, zorder=5)
        seq = int(observation["sequence"]) + 1
        ax.text(distance[index], trajectory[index, 2] + .30,
                f"{seq} {(META_ZH if locale == 'zh' else base.META)[task_id][0]}",
                fontsize=5.15, color=TASK[task_id], ha="center", fontweight="bold",
                fontproperties=FONT_BOLD if locale == "zh" else None)
        previous = index
    ax.set_xlim(0, distance[-1] * 1.06); ax.set_ylim(.55, 7.25)
    ax.set_xticks([0, 10, 20, 30, 40, 48]); ax.set_yticks([1, 3, 5, 7])
    ax.tick_params(axis="both", labelsize=5.0, colors="#82909A", length=1.8, pad=1)
    ax.set_xlabel("已执行三维路径距离 (m)" if locale == "zh" else "executed 3-D path distance (m)",
                  fontsize=5.4, color=MUTED, labelpad=1,
                  fontproperties=FONT if locale == "zh" else None)
    ax.set_ylabel("z (m)", fontsize=5.4, color=MUTED, labelpad=1)
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color("#D3DBE0")
    ax.grid(color="#E6EBEE", lw=.4, alpha=.8)
    clearances = [float(s["bspline_minimum_clearance_m"]) for s in execution["executed_segments"]]
    metrics = ((f"16个可执行视角 | 三维A* + B样条 | 路径 {execution['path_length_m']:.1f} m | "
                f"最小预测净空 {min(clearances):.2f} m | {execution['elapsed_wall_s']:.1f} s") if locale == "zh" else
               (f"16 feasible views  |  3-D A* + B-spline  |  path {execution['path_length_m']:.1f} m  |  "
                f"min. predicted clearance {min(clearances):.2f} m  |  {execution['elapsed_wall_s']:.1f} s"))
    ax.text(.01, -.24, metrics, fontsize=5.7, color=MUTED, transform=ax.transAxes,
            fontproperties=FONT if locale == "zh" else None)


def draw_detection(ax, observation, locale="en"):
    image = plt.imread(RUN / "execution" / observation["rgb"])
    ax.imshow(image)
    ax.set_xticks([]); ax.set_yticks([])
    task_id = observation["task_id"]
    det = observation["verification"]["owlv2"]["detections"][0]
    x1, y1, x2, y2 = det["bbox_xyxy"]
    ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False,
                           edgecolor=TASK[task_id], linewidth=1.7))
    target_name = (META_ZH if locale == "zh" else base.META)[task_id][0]
    ax.text(.035, .055, f"{target_name}  {det['score']:.2f}",
            transform=ax.transAxes, color="white", fontsize=5.35, fontweight="bold",
            va="bottom", ha="left", bbox=dict(fc=TASK[task_id], ec="none", pad=1.3),
            fontproperties=FONT_BOLD if locale == "zh" else None)
    seq = int(observation["sequence"]) + 1
    ax.text(.025, .95, str(seq), transform=ax.transAxes, va="top", color="white",
            fontsize=6.4, fontweight="bold",
            bbox=dict(boxstyle="circle,pad=.20", fc=TASK[task_id], ec="white", lw=.6))
    for spine in ax.spines.values():
        spine.set_color(TASK[task_id]); spine.set_linewidth(1.05)


def add_panel_label(fig, xy, text):
    has_cjk = any(ord(char) > 127 for char in text)
    fig.text(xy[0], xy[1], text, fontsize=8.3, color=INK, fontweight="bold", va="top",
             fontproperties=FONT_BOLD if has_cjk else None)


def render(locale="en"):
    graph, candidates, execution, points, floor_info = base.load_data()
    boxes = load_box_tables()
    scene_graph = read_json(SCENE_GRAPH)

    fig = plt.figure(figsize=(16.6, 8.15), facecolor="white")
    ax_lang = fig.add_axes([.027, .875, .946, .112])
    draw_language_strip(ax_lang, graph, locale)

    ax_memory = fig.add_axes([.018, .205, .205, .575])
    draw_semantic_memory(ax_memory, locale)
    panel = ({
        "memory": "(b) 预探索三维语义记忆", "scene": "(c) 场景中心的任务—视角—路径推理",
        "spatial": "(d) 空间约束与可执行观察视角", "verify": "(f) 开放词汇确认 · 4 / 4",
    } if locale == "zh" else {
        "memory": "(b) Pre-explored 3-D semantic memory", "scene": "(c) Scene-centric task–viewpoint–path reasoning",
        "spatial": "(d) Spatial grounding and executable views", "verify": "(f) Open-vocabulary verification · 4 / 4",
    })
    add_panel_label(fig, (.035, .845), panel["memory"])

    ax_scene = fig.add_axes([.185, .145, .45, .70], projection="3d", computed_zorder=False)
    draw_scene(ax_scene, candidates, execution, points, floor_info, boxes, scene_graph)
    add_panel_label(fig, (.235, .845), panel["scene"])

    ax_local = fig.add_axes([.59, .565, .205, .285], projection="3d", computed_zorder=False)
    draw_spatial_inset(ax_local, candidates, execution, points, boxes, locale)
    add_panel_label(fig, (.60, .845), panel["spatial"])
    ax_quality = fig.add_axes([.61, .475, .175, .155])
    draw_spatial_legend(ax_quality, candidates, execution, locale)

    add_panel_label(fig, (.795, .845), panel["verify"])
    rgb_positions = [(.795, .585), (.895, .585), (.795, .335), (.895, .335)]
    for position, observation in zip(rgb_positions, execution["observations"]):
        ax = fig.add_axes([position[0], position[1], .095, .235])
        draw_detection(ax, observation, locale)

    ax_ribbon = fig.add_axes([.245, .055, .50, .15])
    draw_execution_ribbon(ax_ribbon, execution, floor_info, locale)
    legend = ("浅色视锥：可执行 q · 实色视锥：选中 q* · 彩色曲线：已执行B样条" if locale == "zh" else
              "faint frustums: feasible q   ·   saturated frustums: selected q*   ·   colored curves: executed B-spline")
    fig.text(.485, .237, legend, fontsize=5.55, color=MUTED, ha="center",
             fontproperties=FONT if locale == "zh" else None)

    OUT.mkdir(parents=True, exist_ok=True)
    stem = f"tro_system_overview_{locale}"
    fig.savefig(OUT / f"{stem}.png", dpi=360, facecolor="white")
    fig.savefig(OUT / f"{stem}.pdf", facecolor="white")
    plt.close(fig)
    return stem


def curate(stems):
    SELECTED.mkdir(parents=True, exist_ok=True)
    FINAL.mkdir(parents=True, exist_ok=True)
    for stem in stems:
        for suffix in (".png", ".pdf"):
            shutil.copy2(OUT / f"{stem}{suffix}", SELECTED / f"{stem}{suffix}")
    for suffix in (".png", ".pdf"):
        shutil.copy2(OUT / f"tro_system_overview_en{suffix}",
                     FINAL / f"system_overview_tro{suffix}")
        shutil.copy2(OUT / f"tro_system_overview_zh{suffix}",
                     FINAL / f"system_overview_tro_zh{suffix}")
    manifest = {
        "source_run": str(RUN.relative_to(ROOT)),
        "source_map": str(base.PCD.relative_to(ROOT)),
        "truthfulness": {
            "task_count": 4,
            "feasible_viewpoint_count": 16,
            "executed_order": [base.META[t][0] for t in base.TASKS],
            "active_recovery_executed": False,
            "all_targets_found": True,
        },
        "exports": [f"{stem}{suffix}" for stem in stems for suffix in (".png", ".pdf")],
    }
    (SELECTED / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    stems = [render("en"), render("zh")]
    curate(stems)
    print(f"candidate figures: {OUT}")
    print(f"selected figures:  {SELECTED}")
    print(f"final figure:       {FINAL / 'system_overview_tro.png'}")


if __name__ == "__main__":
    main()
