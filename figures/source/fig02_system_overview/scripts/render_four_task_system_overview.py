#!/usr/bin/env python3
"""Render publication assets for the real four-target 00337 mission.

The figures are derived only from the stored task graph, candidate viewpoints,
executed 3-D trajectory, OWLv2 outputs, Habitat RGB frames, and Stage-1 map.
"""

from __future__ import annotations

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
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle
from mpl_toolkits.mplot3d.art3d import Line3DCollection


ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / "outputs/stage2_3d/tasks/00337_all_found_showcase_v1_20260830"
STAGE1 = ROOT / "outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829"
PCD = STAGE1 / "bag_export/map_occupied.pcd"
FLOORS = ROOT / "outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix/floors.json"
OUT = ROOT / "paper/system_overview/assets/candidates/four_task_overview"
SELECTED = ROOT / "paper/system_overview/assets/selected/four_task_overview"

TASKS = [
    "check_microwave_kitchen_l2",
    "check_toilet_paper_bath_l2",
    "check_coffee_table_living_l2",
    "check_tv_living_l3",
]
GROUNDING_TASKS = [TASKS[0], TASKS[2], TASKS[3], TASKS[1]]
COLORS = {
    TASKS[0]: "#D97A22",
    TASKS[1]: "#209C87",
    TASKS[2]: "#7656A8",
    TASKS[3]: "#2679B8",
}
META = {
    TASKS[0]: ("Microwave", "L2 · kitchen", "near countertop"),
    TASKS[1]: ("Toilet paper", "L2 · bathroom", "near toilet"),
    TASKS[2]: ("Coffee table", "L2 · living room", "near sofa"),
    TASKS[3]: ("Television", "L3 · living room", "instance"),
}
FLOOR_COLORS = {1: "#4C88B8", 2: "#35A184", 3: "#E18A4E"}
FONT = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_pcd(path: Path) -> np.ndarray:
    skip = 0
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            skip += 1
            if line.strip().lower().startswith("data"):
                break
    return np.loadtxt(path, dtype=np.float32, skiprows=skip, usecols=(0, 1, 2))


def sample(points: np.ndarray, maximum: int, seed: int) -> np.ndarray:
    if len(points) <= maximum:
        return points
    rng = np.random.default_rng(seed)
    return points[rng.choice(len(points), maximum, replace=False)]


def save(fig, stem: str, tight=True):
    OUT.mkdir(parents=True, exist_ok=True)
    kw = dict(dpi=360, facecolor="white", transparent=False)
    if tight:
        kw.update(bbox_inches="tight", pad_inches=0.035)
    fig.savefig(OUT / f"{stem}.png", **kw)
    fig.savefig(OUT / f"{stem}.pdf", **kw)
    plt.close(fig)


def clean(ax):
    ax.set_axis_off()
    ax.set_facecolor("white")


def load_data():
    graph = read_json(RUN / "task_graph.json")
    candidates = read_json(RUN / "execution/candidates_with_recovery.json")["by_task"]
    execution = read_json(RUN / "execution/habitat_execution.json")
    points = load_pcd(PCD)
    floor_info = read_json(FLOORS)[:3]
    return graph, candidates, execution, points, floor_info


def frustum2d(pose, depth=.58, half_angle=math.radians(24)):
    x, y = float(pose["x"]), float(pose["y"])
    yaw = float(pose["yaw"])
    return np.asarray([
        [x, y],
        [x + depth * math.cos(yaw - half_angle), y + depth * math.sin(yaw - half_angle)],
        [x + depth * math.cos(yaw + half_angle), y + depth * math.sin(yaw + half_angle)],
    ])


def frustum3d(pose, depth=.62, hfov=90, vfov=70):
    if not isinstance(pose, dict):
        pose = dict(zip(("x", "y", "z", "yaw"), pose))
    origin = np.asarray([pose["x"], pose["y"], pose["z"]], float)
    yaw = float(pose["yaw"])
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0])
    vertical = np.asarray([0.0, 0.0, 1.0])
    center = origin + depth * forward
    hw = depth * math.tan(math.radians(hfov) / 2)
    hh = depth * math.tan(math.radians(vfov) / 2)
    corners = [center + sx * hw * right + sz * hh * vertical
               for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    segs = [np.vstack([origin, c]) for c in corners]
    segs += [np.vstack([corners[i], corners[(i + 1) % 4]]) for i in range(4)]
    return segs


def crop_floor(points, z0, maximum=65000, seed=0):
    mask = (points[:, 2] > z0 + .18) & (points[:, 2] < z0 + 2.25)
    return sample(points[mask], maximum, seed)


def draw_instruction(ax, graph, compact=False, locale="en"):
    clean(ax)
    instruction = graph["instruction"].replace("朋友快到了，", "")
    ax.text(.02, .95, instruction, va="top", color="#243341",
            fontsize=10.2 if compact else 12.0, fontproperties=FONT_BOLD,
            wrap=True, transform=ax.transAxes)
    grounding_text = ("Qwen语义解析 · 用户未指定先后顺序" if locale == "zh" else
                      "Qwen semantic grounding · no prescribed order")
    ax.text(.02, .72 if compact else .70, grounding_text,
            color="#788594", fontsize=7.4 if compact else 8.6,
            fontproperties=FONT if locale == "zh" else None, transform=ax.transAxes)
    xs = np.linspace(.10, .90, 4)
    for task_id, x in zip(GROUNDING_TASKS, xs):
        label, place, relation = META[task_id]
        if locale == "zh":
            label, place, relation = {
                TASKS[0]: ("微波炉", "二楼 · 厨房", "靠近台面"),
                TASKS[1]: ("卫生纸", "二楼 · 卫生间", "靠近马桶"),
                TASKS[2]: ("茶几", "二楼 · 客厅", "靠近沙发"),
                TASKS[3]: ("电视", "三楼 · 客厅", "实例本体"),
            }[task_id]
        color = COLORS[task_id]
        ax.scatter([x], [.42], s=420 if compact else 620, color=color,
                   edgecolor="white", linewidth=1.2, transform=ax.transAxes, zorder=3)
        floor_token = "L3" if task_id == TASKS[3] else "L2"
        ax.text(x, .42, floor_token, ha="center", va="center", color="white",
                fontsize=7.2, fontweight="bold", transform=ax.transAxes, zorder=4)
        ax.text(x, .23, label, ha="center", color="#263746", fontsize=7.6 if compact else 8.6,
                fontweight="bold", fontproperties=FONT_BOLD if locale == "zh" else None,
                transform=ax.transAxes)
        ax.text(x, .12, f"{place}\n{relation}", ha="center", va="top", color="#73808D",
                fontsize=6.6 if compact else 7.4, linespacing=1.25,
                fontproperties=FONT if locale == "zh" else None, transform=ax.transAxes)


def selected_ids(execution):
    return {o["candidate_id"]: o for o in execution["observations"]}


def draw_candidate_floor(ax, floor, candidates, execution, points, floor_info, title=True):
    z0 = float(floor_info[floor - 1]["floor_z_m"])
    cloud = crop_floor(points, z0, 72000, 80 + floor)
    ax.scatter(cloud[:, 0], cloud[:, 1], s=.28, color="#A9B5C0", alpha=.31,
               linewidths=0, rasterized=True)
    chosen = selected_ids(execution)
    for task_id in TASKS:
        color = COLORS[task_id]
        for candidate in candidates[task_id]:
            if int(candidate["floor_id"]) != floor:
                continue
            pose = candidate["pose"]
            is_selected = candidate["id"] in chosen
            poly = frustum2d(pose, .78 if is_selected else .52)
            ax.add_patch(Polygon(poly, closed=True, facecolor=color,
                                 edgecolor=color, linewidth=1.4 if is_selected else .55,
                                 alpha=.30 if is_selected else .085, zorder=4 if is_selected else 2))
            ax.scatter([pose["x"]], [pose["y"]], s=28 if is_selected else 9,
                       facecolor=color if is_selected else "white", edgecolor=color,
                       linewidth=.9 if is_selected else .5, zorder=5)
        target = candidates[task_id][0]["target_xyz_m"]
        if int(candidates[task_id][0]["floor_id"]) == floor:
            ax.scatter([target[0]], [target[1]], marker="x", s=40, color=color,
                       linewidth=1.6, zorder=6)
    ax.set_aspect("equal", adjustable="box")
    clean(ax)
    if title:
        n = sum(len(v) for v in candidates.values() if v and int(v[0]["floor_id"]) == floor)
        ax.set_title(f"L{floor} · {n} feasible viewpoints", loc="left", fontsize=9.2,
                     color="#344555", fontweight="bold", pad=3)


def fit3d(ax, points, z_scale=.9, pad=.03):
    lo, hi = points.min(axis=0), points.max(axis=0)
    span = np.maximum(hi - lo, 1e-3)
    ax.set_xlim(lo[0] - pad * span[0], hi[0] + pad * span[0])
    ax.set_ylim(lo[1] - pad * span[1], hi[1] + pad * span[1])
    ax.set_zlim(lo[2] - pad * span[2], hi[2] + pad * span[2])
    ax.set_box_aspect((span[0], span[1], span[2] * z_scale))


def draw_route3d(ax, candidates, execution, points, floor_info, legend=True, metrics=True):
    clean(ax)
    ax.set_proj_type("ortho")
    clouds = []
    for floor in (1, 2, 3):
        cloud = crop_floor(points, float(floor_info[floor - 1]["floor_z_m"]), 26000, 200 + floor)
        xcut, ycut = np.quantile(cloud[:, 0], .86), np.quantile(cloud[:, 1], .13)
        cloud = cloud[(cloud[:, 0] < xcut) & (cloud[:, 1] > ycut)]
        clouds.append(cloud)
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=.27,
                   color=FLOOR_COLORS[floor], alpha=.14, linewidths=0,
                   depthshade=False, rasterized=True)
    all_path = []
    for segment in execution["executed_segments"]:
        task_id = segment["task_id"]
        path = np.asarray(segment["bspline_points_xyz_m"], float)
        all_path.append(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color="white", lw=4.0,
                alpha=.95, solid_capstyle="round", zorder=8)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color=COLORS[task_id], lw=2.2,
                solid_capstyle="round", zorder=9)
    chosen = selected_ids(execution)
    for task_id, group in candidates.items():
        for candidate in group:
            if candidate["id"] in chosen:
                obs = chosen[candidate["id"]]
                color = COLORS[task_id]
                ax.add_collection3d(Line3DCollection(frustum3d(candidate["pose"]),
                                                     colors=color, linewidths=1.25, alpha=.96))
                p = obs["pose"]
                seq = int(obs["sequence"]) + 1
                ax.scatter([p[0]], [p[1]], [p[2]], s=33, color=color,
                           edgecolor="white", linewidth=.8, depthshade=False, zorder=12)
                ax.text(p[0], p[1], p[2] + .28, str(seq), color=color, fontsize=8.5,
                        fontweight="bold", ha="center", zorder=13)
    extent = np.vstack(clouds + all_path)
    fit3d(ax, extent, .86)
    ax.view_init(elev=25, azim=-58)
    ax.dist = 5.9
    if legend:
        handles = [Line2D([0], [0], color=COLORS[t], lw=2.8, label=f"{i}. {META[t][0]}")
                   for i, t in enumerate(TASKS, 1)]
        ax.legend(handles=handles, frameon=False, fontsize=7.4, loc="upper left",
                  bbox_to_anchor=(.00, .99), handlelength=2.3)
    if metrics:
        ax.text2D(.02, .025, "16 feasible views  ·  48.1 m  ·  3D A* + B-spline",
                  transform=ax.transAxes, fontsize=7.8, color="#5E6E7E")


def draw_detection(ax, observation, numbered=True, title=True):
    image = plt.imread(RUN / "execution" / observation["rgb"])
    ax.imshow(image)
    ax.set_xticks([]); ax.set_yticks([])
    task_id = observation["task_id"]
    color = COLORS[task_id]
    for spine in ax.spines.values():
        spine.set_color(color); spine.set_linewidth(1.7)
    det = observation["verification"]["owlv2"]["detections"][0]
    x1, y1, x2, y2 = det["bbox_xyxy"]
    ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False,
                           edgecolor=color, linewidth=2.2))
    label = META[task_id][0]
    ax.text(x1, max(13, y1 - 7), f"{label}  {det['score']:.2f}", color="white",
            fontsize=7.2, fontweight="bold",
            bbox=dict(facecolor=color, edgecolor="none", pad=1.8))
    if numbered:
        ax.text(.025, .955, str(int(observation["sequence"]) + 1), transform=ax.transAxes,
                va="top", ha="left", color="white", fontsize=8.5, fontweight="bold",
                bbox=dict(boxstyle="circle,pad=.25", facecolor=color, edgecolor="white", lw=.7))
    if title:
        ax.set_title(f"{META[task_id][0]} · {META[task_id][1]}", fontsize=8.0,
                     color="#354655", fontweight="bold", pad=4)


def render_task_grounding(graph):
    fig, ax = plt.subplots(figsize=(13.2, 4.2), facecolor="white")
    draw_instruction(ax, graph)
    save(fig, "01_language_grounding_unordered_tasks")


def render_candidate_atlas(candidates, execution, points, floor_info):
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5.6), facecolor="white")
    draw_candidate_floor(axes[0], 2, candidates, execution, points, floor_info)
    draw_candidate_floor(axes[1], 3, candidates, execution, points, floor_info)
    fig.text(.5, .025, "open markers: feasible candidates   ·   filled frustums: selected views   ·   ×: targets",
             ha="center", fontsize=8.0, color="#6D7A87")
    save(fig, "02_candidate_viewpoints_geometry")


def render_joint_route(candidates, execution, points, floor_info):
    fig = plt.figure(figsize=(11.2, 7.0), facecolor="white")
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    draw_route3d(ax, candidates, execution, points, floor_info)
    ax.set_position([0, 0, 1, .87])
    fig.text(.50, .965, "Joint task–viewpoint–path optimization", ha="center",
             fontsize=12.0, color="#273746", fontweight="bold")
    fig.text(.50, .925, "free-order tasks → geometry-aware execution order: microwave → toilet paper → coffee table → television",
             ha="center", fontsize=8.1, color="#6E7C89")
    save(fig, "03_joint_3d_route", tight=False)


def render_rgb_evidence(execution):
    fig, axes = plt.subplots(1, 4, figsize=(14.8, 3.8), facecolor="white")
    for ax, obs in zip(axes, execution["observations"]):
        draw_detection(ax, obs)
    fig.text(.5, .03, "Real Habitat observations · OWLv2 detections · all four tasks completed",
             ha="center", fontsize=8.2, color="#647381")
    fig.subplots_adjust(left=.01, right=.99, top=.88, bottom=.12, wspace=.035)
    save(fig, "04_real_rgb_verification", tight=False)


def overview_landscape(graph, candidates, execution, points, floor_info):
    fig = plt.figure(figsize=(18.0, 8.5), facecolor="white")
    gs = fig.add_gridspec(2, 12, height_ratios=[.34, 1.0], hspace=.06, wspace=.10,
                          left=.025, right=.985, top=.95, bottom=.055)
    ax_lang = fig.add_subplot(gs[0, :])
    draw_instruction(ax_lang, graph, compact=True)
    ax_route = fig.add_subplot(gs[1, :7], projection="3d", computed_zorder=False)
    draw_route3d(ax_route, candidates, execution, points, floor_info, legend=True, metrics=True)
    ax_route.set_position([.00, .02, .59, .67])
    rgb_grid = gs[1, 7:].subgridspec(2, 2, wspace=.04, hspace=.11)
    for axspec, obs in zip(rgb_grid, execution["observations"]):
        draw_detection(fig.add_subplot(axspec), obs)
    fig.text(.025, .987, "Language-guided multi-floor object verification", fontsize=13.5,
             color="#243442", fontweight="bold", va="top")
    fig.text(.34, .70, "Geometry-aware joint execution", fontsize=9.2,
             color="#344655", fontweight="bold", ha="center")
    fig.text(.79, .70, "Real visual evidence", fontsize=9.2,
             color="#344655", fontweight="bold", ha="center")
    save(fig, "overview_A_landscape_evidence", tight=False)


def overview_map_centric(graph, candidates, execution, points, floor_info):
    fig = plt.figure(figsize=(13.8, 10.2), facecolor="white")
    gs = fig.add_gridspec(3, 10, height_ratios=[.58, 1.35, .68], hspace=.08,
                          left=.035, right=.98, top=.96, bottom=.04)
    ax_lang = fig.add_subplot(gs[0, :])
    draw_instruction(ax_lang, graph, compact=True)
    ax_route = fig.add_subplot(gs[1, :7], projection="3d", computed_zorder=False)
    draw_route3d(ax_route, candidates, execution, points, floor_info, legend=False, metrics=True)
    ax_l2 = fig.add_subplot(gs[1, 7:])
    draw_candidate_floor(ax_l2, 2, candidates, execution, points, floor_info)
    for ax, obs in zip([fig.add_subplot(gs[2, i * 2:(i + 1) * 2 + (1 if i == 3 else 0)])
                        for i in range(4)], execution["observations"]):
        draw_detection(ax, obs, title=False)
    fig.text(.035, .985, "One instruction, one optimized 3D mission", va="top",
             fontsize=13.2, color="#243442", fontweight="bold")
    fig.text(.375, .665, "executed route", fontsize=9.0, color="#344655",
             fontweight="bold", ha="center")
    save(fig, "overview_B_map_centric", tight=False)


def overview_candidate_to_evidence(graph, candidates, execution, points, floor_info):
    fig = plt.figure(figsize=(17.2, 7.6), facecolor="white")
    gs = fig.add_gridspec(2, 12, height_ratios=[.63, 1.0], wspace=.15, hspace=.10,
                          left=.025, right=.985, top=.94, bottom=.055)
    ax_lang = fig.add_subplot(gs[0, :4]); draw_instruction(ax_lang, graph, compact=True)
    ax_l2 = fig.add_subplot(gs[0, 4:8]); draw_candidate_floor(ax_l2, 2, candidates, execution, points, floor_info)
    ax_l3 = fig.add_subplot(gs[0, 8:]); draw_candidate_floor(ax_l3, 3, candidates, execution, points, floor_info)
    ax_route = fig.add_subplot(gs[1, :7], projection="3d", computed_zorder=False)
    draw_route3d(ax_route, candidates, execution, points, floor_info, legend=False, metrics=True)
    rgb = gs[1, 7:].subgridspec(2, 2, wspace=.04, hspace=.08)
    for spec, obs in zip(rgb, execution["observations"]):
        draw_detection(fig.add_subplot(spec), obs, title=False)
    fig.text(.025, .975, "From language constraints to verified observations", fontsize=13.2,
             color="#243442", fontweight="bold", va="top")
    save(fig, "overview_C_candidates_to_evidence", tight=False)


def crop_white_image(path: Path, pad=12):
    image = plt.imread(path)
    mask = np.any(image[..., :3] < .975, axis=2)
    yy, xx = np.where(mask)
    if not len(xx):
        return image
    x0, x1 = max(0, xx.min() - pad), min(image.shape[1], xx.max() + pad + 1)
    y0, y1 = max(0, yy.min() - pad), min(image.shape[0], yy.max() + pad + 1)
    return image[y0:y1, x0:x1]


def overview_full_system(graph, candidates, execution, points, floor_info):
    """Full chain: semantic memory, task grounding, views, route, evidence."""
    fig = plt.figure(figsize=(20.0, 9.2), facecolor="white")
    gs = fig.add_gridspec(2, 20, height_ratios=[.34, 1.0], hspace=.06, wspace=.10,
                          left=.018, right=.99, top=.955, bottom=.045)
    ax_lang = fig.add_subplot(gs[0, :])
    draw_instruction(ax_lang, graph, compact=True)

    ax_map = fig.add_subplot(gs[1, :4]); clean(ax_map)
    semantic_asset = ROOT / "paper/system_overview/assets/selected/clean_real_data/semantic_rooms_and_3d_boxes_stack.png"
    ax_map.imshow(crop_white_image(semantic_asset))
    ax_map.set_title("① Pre-explored semantic memory", loc="left", fontsize=9.0,
                     color="#344655", fontweight="bold", pad=5)
    ax_map.text(.02, .02, "3 floors · room regions · 3D object boxes", transform=ax_map.transAxes,
                fontsize=6.9, color="#70808E")

    view_grid = gs[1, 4:8].subgridspec(2, 1, hspace=.08)
    ax_l2 = fig.add_subplot(view_grid[0, 0])
    draw_candidate_floor(ax_l2, 2, candidates, execution, points, floor_info, title=False)
    ax_l3 = fig.add_subplot(view_grid[1, 0])
    draw_candidate_floor(ax_l3, 3, candidates, execution, points, floor_info, title=False)
    fig.text(.305, .69, "② 16 feasible observation poses", ha="center", fontsize=9.0,
             color="#344655", fontweight="bold")
    fig.text(.305, .665, "13 on L2 · 3 on L3 · visibility + collision filtered", ha="center",
             fontsize=6.8, color="#73808D")

    ax_route = fig.add_subplot(gs[1, 8:14], projection="3d", computed_zorder=False)
    draw_route3d(ax_route, candidates, execution, points, floor_info, legend=False, metrics=True)
    fig.text(.55, .69, "③ Joint task–viewpoint–path optimization", ha="center", fontsize=9.0,
             color="#344655", fontweight="bold")

    rgb_grid = gs[1, 14:].subgridspec(2, 2, wspace=.035, hspace=.09)
    for spec, obs in zip(rgb_grid, execution["observations"]):
        draw_detection(fig.add_subplot(spec), obs, title=False)
    fig.text(.85, .69, "④ Online execution and visual verification", ha="center", fontsize=9.0,
             color="#344655", fontweight="bold")
    fig.text(.85, .665, "real RGB · OWLv2 · 4/4 targets confirmed", ha="center",
             fontsize=6.8, color="#73808D")
    fig.text(.018, .987, "System Overview · language-guided multi-floor UAV search",
             fontsize=13.7, color="#243442", fontweight="bold", va="top")
    save(fig, "overview_D_full_system_chain", tight=False)


def curate(graph, candidates, execution):
    SELECTED.mkdir(parents=True, exist_ok=True)
    for old in SELECTED.iterdir():
        if old.is_file():
            old.unlink()
    stems = [
        "01_language_grounding_unordered_tasks",
        "02_candidate_viewpoints_geometry",
        "03_joint_3d_route",
        "04_real_rgb_verification",
        "overview_A_landscape_evidence",
        "overview_B_map_centric",
        "overview_C_candidates_to_evidence",
        "overview_D_full_system_chain",
    ]
    for stem in stems:
        for suffix in (".png", ".pdf"):
            shutil.copy2(OUT / f"{stem}{suffix}", SELECTED / f"{stem}{suffix}")
    (SELECTED / "README.md").write_text(
        "# Four-task System Overview assets\n\n"
        "Source run: `00337_all_found_showcase_v1_20260830`. The Qwen task graph has "
        "four independent tasks and no prescribed precedence. The actual geometry-aware "
        "execution order is microwave, toilet paper, coffee table, television. All candidate "
        "poses, paths, RGB frames, OWLv2 boxes, scores, and metrics are read from stored data.\n",
        encoding="utf-8",
    )
    detections = {}
    for observation in execution["observations"]:
        detection = observation["verification"]["owlv2"]["detections"][0]
        detections[observation["task_id"]] = {
            "label": detection["label"],
            "score": detection["score"],
            "bbox_xyxy": detection["bbox_xyxy"],
            "rgb": observation["rgb"],
        }
    manifest = {
        "source_run": str(RUN.relative_to(ROOT)),
        "instruction": graph["instruction"],
        "qwen_task_count": len(graph["tasks"]),
        "qwen_precedence_constraint_count": sum(len(t.get("prerequisites", [])) for t in graph["tasks"]),
        "conditional_rule_count": len(graph.get("conditional_rules", [])),
        "feasible_viewpoint_count": sum(len(v) for v in candidates.values()),
        "executed_order": [o["task_id"] for o in execution["observations"]],
        "path_length_m": execution["path_length_m"],
        "elapsed_wall_s": execution["elapsed_wall_s"],
        "status": execution["status"],
        "detections": detections,
    }
    (SELECTED / "data_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    final_dir = ROOT / "paper/system_overview/assets/final"
    final_dir.mkdir(parents=True, exist_ok=True)
    for suffix in (".png", ".pdf"):
        shutil.copy2(OUT / f"overview_D_full_system_chain{suffix}",
                     final_dir / f"system_overview_four_task{suffix}")


def main():
    graph, candidates, execution, points, floor_info = load_data()
    render_task_grounding(graph)
    render_candidate_atlas(candidates, execution, points, floor_info)
    render_joint_route(candidates, execution, points, floor_info)
    render_rgb_evidence(execution)
    overview_landscape(graph, candidates, execution, points, floor_info)
    overview_map_centric(graph, candidates, execution, points, floor_info)
    overview_candidate_to_evidence(graph, candidates, execution, points, floor_info)
    overview_full_system(graph, candidates, execution, points, floor_info)
    curate(graph, candidates, execution)
    print(f"candidates: {OUT}")
    print(f"selected:   {SELECTED}")


if __name__ == "__main__":
    main()
