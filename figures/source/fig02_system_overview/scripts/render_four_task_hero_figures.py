#!/usr/bin/env python3
"""Create compact, publication-oriented hero/method figures for the four-task run."""

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
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

import render_four_task_system_overview as base


ROOT = base.ROOT
OUT = ROOT / "paper/system_overview/assets/candidates/four_task_hero"
SELECTED = ROOT / "paper/system_overview/assets/selected/four_task_hero"
FINAL = ROOT / "paper/system_overview/assets/final"
SEMANTIC_MAP = ROOT / "paper/system_overview/assets/selected/clean_real_data/semantic_rooms_and_3d_boxes_stack.png"
L2_BOXES = ROOT / "outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix/L2_boxes.csv"
MICROWAVE_TASK = base.TASKS[0]


def save(fig, stem, tight=False):
    OUT.mkdir(parents=True, exist_ok=True)
    kwargs = dict(dpi=360, facecolor="white", transparent=False)
    if tight:
        kwargs.update(bbox_inches="tight", pad_inches=.035)
    fig.savefig(OUT / f"{stem}.png", **kwargs)
    fig.savefig(OUT / f"{stem}.pdf", **kwargs)
    plt.close(fig)


def box_row(index):
    with L2_BOXES.open("r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    row = rows[index]
    return {
        "center": np.asarray([float(row[f"t{a}_world_object"]) for a in "xyz"]),
        "size": np.asarray([float(row[f"scale_{a}"]) for a in "xyz"]),
        "yaw": 2 * math.atan2(float(row["qz_world_object"]), float(row["qw_world_object"])),
        "name": row["name"],
    }


def box_geometry(center, size, yaw=0.0):
    signs = np.asarray([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ], float)
    points = signs * size[None, :] / 2
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    points = points @ rotation.T + center
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
             [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return [points[list(edge)] for edge in edges], [points[face] for face in faces]


def add_box(ax, center, size, yaw, color, alpha=.08, linewidth=1.2):
    edges, faces = box_geometry(np.asarray(center), np.asarray(size), yaw)
    ax.add_collection3d(Poly3DCollection(faces, facecolors=color, edgecolors="none", alpha=alpha))
    ax.add_collection3d(Line3DCollection(edges, colors=color, linewidths=linewidth, alpha=.9))


def draw_local_geometry(ax, candidates, execution, points, legend=True):
    base.clean(ax); ax.set_proj_type("ortho")
    anchor, target = box_row(30), box_row(32)
    center = (anchor["center"] + target["center"]) / 2
    mask = ((np.abs(points[:, 0] - center[0]) < 2.5) &
            (np.abs(points[:, 1] - center[1]) < 2.8) &
            (points[:, 2] > 2.7) & (points[:, 2] < 4.75))
    cloud = base.sample(points[mask], 38000, 811)
    height_color = plt.get_cmap("Blues")((cloud[:, 2] - cloud[:, 2].min()) /
                                           max(np.ptp(cloud[:, 2]), 1e-3))
    ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=.42, c=height_color,
               alpha=.19, linewidths=0, depthshade=False, rasterized=True)
    add_box(ax, anchor["center"], anchor["size"], anchor["yaw"], "#596B7B", .08, 1.25)
    add_box(ax, target["center"], target["size"], target["yaw"], base.COLORS[MICROWAVE_TASK], .19, 1.6)
    first = candidates[MICROWAVE_TASK][0]
    search_center = np.asarray(first["search_region_center_xyz_m"])
    search_size = np.asarray(first["search_region_size_xyz_m"])
    add_box(ax, search_center, search_size, anchor["yaw"], "#D6A23F", .14, 1.35)
    selected = execution["observations"][0]["candidate_id"]
    for candidate in candidates[MICROWAVE_TASK]:
        chosen = candidate["id"] == selected
        ax.add_collection3d(Line3DCollection(
            base.frustum3d(candidate["pose"], .72 if chosen else .48),
            colors=base.COLORS[MICROWAVE_TASK],
            linewidths=1.35 if chosen else .42,
            alpha=.96 if chosen else .18,
        ))
    extent = np.vstack([cloud, anchor["center"], target["center"]] +
                       [np.asarray([[c["pose"][k] for k in ("x", "y", "z")]])
                        for c in candidates[MICROWAVE_TASK]])
    base.fit3d(ax, extent, .90, .025)
    ax.view_init(elev=24, azim=-54); ax.dist = 4.8
    ax.text(anchor["center"][0], anchor["center"][1], anchor["center"][2] - .18,
            "countertop", color="#516474", fontsize=6.7, ha="center")
    ax.text(target["center"][0], target["center"][1], target["center"][2] + .22,
            "microwave", color=base.COLORS[MICROWAVE_TASK], fontsize=6.7,
            ha="center", fontweight="bold")
    if legend:
        handles = [
            Line2D([0], [0], color="#596B7B", lw=2, label="countertop anchor"),
            Line2D([0], [0], color="#D6A23F", lw=2, label="near search region"),
            Line2D([0], [0], color=base.COLORS[MICROWAVE_TASK], lw=2, label="microwave / viewpoints"),
        ]
        ax.legend(handles=handles, frameon=False, fontsize=7.0, loc="lower left",
                  bbox_to_anchor=(.00, .01), handlelength=2.0)


def draw_candidate_quality(ax, candidates, execution):
    group = candidates[MICROWAVE_TASK]
    chosen = execution["observations"][0]["candidate_id"]
    coverage = np.asarray([c["view_quality"] for c in group])
    costs = np.asarray([c["terminal_cost"] for c in group])
    order = np.argsort(costs)
    x = np.arange(1, len(group) + 1)
    for rank, idx in enumerate(order):
        is_selected = group[idx]["id"] == chosen
        color = base.COLORS[MICROWAVE_TASK] if is_selected else "#AAB5BF"
        ax.plot([x[rank], x[rank]], [0, coverage[idx]], color=color,
                lw=2.1 if is_selected else 1.0, alpha=.95)
        ax.scatter([x[rank]], [coverage[idx]], s=54 if is_selected else 25,
                   color=color, edgecolor="white", linewidth=.7, zorder=3)
        ax.text(x[rank], coverage[idx] + .055, f"{int(round(coverage[idx] * 8))}/8",
                ha="center", fontsize=6.7, color=color, fontweight="bold" if is_selected else None)
    ax.set_ylim(0, .72); ax.set_xlim(.5, len(group) + .5)
    ax.set_xticks(x, [f"v{i}" for i in x], fontsize=6.7, color="#677481")
    ax.set_yticks([0, .25, .5], ["0", "25", "50%"], fontsize=6.6, color="#7A8692")
    ax.set_ylabel("visible search-region samples", fontsize=7.0, color="#667583")
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.spines["bottom"].set_color("#D7DDE2")
    ax.grid(axis="y", color="#E7EBEF", lw=.55)
    ax.set_title("Executable viewpoint ranking", loc="left", fontsize=9.0,
                 color="#344555", fontweight="bold", pad=6)
    ax.text(1.0, .63, "selected", ha="center", va="bottom",
            fontsize=6.8, color=base.COLORS[MICROWAVE_TASK], fontweight="bold")


def trajectory_profile(execution):
    trajectory = np.asarray(execution["trajectory_xyz_yaw"], float)
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(trajectory[:, :3], axis=0), axis=1))]
    obs_distance = []
    for observation in execution["observations"]:
        pose = np.asarray(observation["pose"][:3])
        index = np.argmin(np.linalg.norm(trajectory[:, :3] - pose, axis=1))
        obs_distance.append(distance[index])
    return trajectory, distance, np.asarray(obs_distance)


def draw_height_profile(ax, execution, floor_info, compact=False):
    trajectory, distance, obs_distance = trajectory_profile(execution)
    for floor, info in enumerate(floor_info, 1):
        z0 = float(info["floor_z_m"])
        ax.axhspan(z0 + .45, z0 + 1.65, color=base.FLOOR_COLORS[floor], alpha=.055)
        ax.text(distance[-1] * 1.005, z0 + 1.02, f"L{floor}", color=base.FLOOR_COLORS[floor],
                fontsize=6.8, va="center", fontweight="bold")
    ax.plot(distance, trajectory[:, 2], color="#465A6C", lw=1.45)
    for index, (task_id, d) in enumerate(zip(base.TASKS, obs_distance), 1):
        z = np.interp(d, distance, trajectory[:, 2])
        ax.scatter([d], [z], s=31, color=base.COLORS[task_id], edgecolor="white", lw=.7, zorder=4)
        ax.text(d, z + .20, str(index), color=base.COLORS[task_id], fontsize=6.8,
                ha="center", fontweight="bold")
    ax.set_xlim(0, distance[-1] * 1.06); ax.set_ylim(0, 8.2)
    ax.set_xlabel("executed distance (m)", fontsize=7.0, color="#657380")
    ax.set_ylabel("UAV z (m)", fontsize=7.0, color="#657380")
    ax.tick_params(labelsize=6.6, colors="#788490", length=2)
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color("#D7DDE2")
    ax.grid(color="#E8ECEF", lw=.5, alpha=.8)
    if not compact:
        ax.set_title("Continuous inter-floor execution", loc="left", fontsize=9.0,
                     color="#344555", fontweight="bold", pad=6)


def draw_rgb_grid(fig, spec, execution, titles=False):
    grid = spec.subgridspec(2, 2, wspace=.035, hspace=.075)
    axes = []
    for cell, observation in zip(grid, execution["observations"]):
        ax = fig.add_subplot(cell)
        base.draw_detection(ax, observation, title=titles)
        axes.append(ax)
    return axes


def draw_semantic_memory(ax):
    base.clean(ax)
    ax.imshow(base.crop_white_image(SEMANTIC_MAP))
    ax.text(.02, .03, "3 floors · rooms · 3D object boxes", transform=ax.transAxes,
            fontsize=6.8, color="#6D7B88")


def render_spatial_microscope(candidates, execution, points):
    fig = plt.figure(figsize=(13.6, 5.4), facecolor="white")
    gs = fig.add_gridspec(1, 12, left=.02, right=.99, top=.90, bottom=.10, wspace=.24)
    ax_geo = fig.add_subplot(gs[0, :6], projection="3d", computed_zorder=False)
    draw_local_geometry(ax_geo, candidates, execution, points)
    ax_quality = fig.add_subplot(gs[0, 6:9]); draw_candidate_quality(ax_quality, candidates, execution)
    ax_rgb = fig.add_subplot(gs[0, 9:]); base.draw_detection(ax_rgb, execution["observations"][0], title=False)
    fig.text(.02, .965, "Spatially grounded observation planning · microwave near the countertop",
             fontsize=12.0, color="#263746", fontweight="bold", va="top")
    fig.text(.02, .925, "anchor geometry → near-region sampling → collision/visibility filtering → selected view → visual confirmation",
             fontsize=7.6, color="#73808C", va="top")
    save(fig, "spatial_relation_microscope", tight=False)


def render_motion_profile(execution, floor_info):
    fig, ax = plt.subplots(figsize=(10.8, 3.2), facecolor="white")
    draw_height_profile(ax, execution, floor_info)
    fig.subplots_adjust(left=.075, right=.95, top=.82, bottom=.22)
    save(fig, "cross_floor_height_profile", tight=False)


def render_hero(language, candidates, execution, points, floor_info, locale, clean_title=False):
    fig = plt.figure(figsize=(18.4, 8.0), facecolor="white")
    gs = fig.add_gridspec(2, 18, height_ratios=[.27, 1.0], left=.018, right=.992,
                          top=.94 if not clean_title else .98, bottom=.045,
                          hspace=.035, wspace=.08)
    ax_instruction = fig.add_subplot(gs[0, :]); base.draw_instruction(ax_instruction, language, compact=True, locale=locale)
    ax_map = fig.add_subplot(gs[1, :4]); draw_semantic_memory(ax_map)
    ax_route = fig.add_subplot(gs[1, 4:12], projection="3d", computed_zorder=False)
    base.draw_route3d(ax_route, candidates, execution, points, floor_info, legend=True, metrics=False)
    ax_profile = fig.add_axes([.29, .065, .31, .145], facecolor="white")
    draw_height_profile(ax_profile, execution, floor_info, compact=True)
    draw_rgb_grid(fig, gs[1, 12:], execution, titles=False)
    if not clean_title:
        title = ("语言引导的多楼层无人机目标确认" if locale == "zh" else
                 "Language-guided multi-floor UAV object verification")
        subtitle = ("三维语义记忆 · 联合任务—视角—路径优化 · 开放词汇视觉确认" if locale == "zh" else
                    "3D semantic memory · joint task–viewpoint–path optimization · open-vocabulary confirmation")
        fig.text(.018, .985, title, fontsize=13.4, color="#243442", fontweight="bold",
                 fontproperties=base.FONT_BOLD if locale == "zh" else None, va="top")
        fig.text(.018, .956, subtitle, fontsize=7.5, color="#7A8793",
                 fontproperties=base.FONT if locale == "zh" else None, va="top")
    section_labels = (["① 三维语义记忆", "② 优化后的连续三维执行", "③ 真实视觉证据 · 4/4确认"]
                      if locale == "zh" else
                      ["① Semantic memory", "② Optimized continuous 3D execution",
                       "③ Real visual evidence · 4/4 confirmed"])
    fig.text(.11, .72, section_labels[0], fontsize=8.6, color="#344655", fontweight="bold",
             fontproperties=base.FONT_BOLD if locale == "zh" else None, ha="center")
    fig.text(.45, .72, section_labels[1], fontsize=8.6, color="#344655", fontweight="bold",
             fontproperties=base.FONT_BOLD if locale == "zh" else None, ha="center")
    fig.text(.83, .72, section_labels[2], fontsize=8.6, color="#344655", fontweight="bold",
             fontproperties=base.FONT_BOLD if locale == "zh" else None, ha="center")
    stem = "hero_visual_" + ("clean" if clean_title else locale)
    save(fig, stem, tight=False)


def render_method(language, candidates, execution, points, floor_info, locale, clean_title=False):
    fig = plt.figure(figsize=(18.8, 10.2), facecolor="white")
    gs = fig.add_gridspec(3, 18, height_ratios=[.28, .82, .90], left=.018, right=.992,
                          top=.94 if not clean_title else .98, bottom=.035,
                          hspace=.08, wspace=.12)
    ax_instruction = fig.add_subplot(gs[0, :]); base.draw_instruction(ax_instruction, language, compact=True, locale=locale)
    ax_memory = fig.add_subplot(gs[1, :4]); draw_semantic_memory(ax_memory)
    method_titles = (["① 语义地图", "② 空间约束实例化", "③ 候选质量", "④ 可执行观察位姿 · 二/三楼",
                      "⑤ 联合三维执行", "⑥ 跨楼层运动", "⑦ RGB与开放词汇确认"]
                     if locale == "zh" else
                     ["① Semantic memory", "② Spatial grounding", "③ Candidate quality",
                      "④ Feasible viewpoints · L2 / L3", "⑤ Joint 3D execution",
                      "⑥ Cross-floor motion", "⑦ RGB + open-vocabulary"])
    title_font = base.FONT_BOLD if locale == "zh" else None
    ax_memory.set_title(method_titles[0], loc="left", fontsize=8.0,
                        color="#405160", fontweight="bold", fontproperties=title_font, pad=4)
    ax_local = fig.add_subplot(gs[1, 4:10], projection="3d", computed_zorder=False)
    draw_local_geometry(ax_local, candidates, execution, points, legend=False)
    ax_local.set_title(method_titles[1], loc="left", fontsize=8.0,
                       color="#405160", fontweight="bold", fontproperties=title_font, pad=4)
    ax_quality = fig.add_subplot(gs[1, 10:13]); draw_candidate_quality(ax_quality, candidates, execution)
    ax_quality.set_title(method_titles[2], loc="left", fontsize=8.0,
                         color="#405160", fontweight="bold", fontproperties=title_font, pad=4)
    candidate_grid = gs[1, 13:].subgridspec(2, 1, hspace=.05)
    ax_l2 = fig.add_subplot(candidate_grid[0, 0]); base.draw_candidate_floor(ax_l2, 2, candidates, execution, points, floor_info, title=False)
    ax_l3 = fig.add_subplot(candidate_grid[1, 0]); base.draw_candidate_floor(ax_l3, 3, candidates, execution, points, floor_info, title=False)
    ax_l2.set_title(method_titles[3], loc="left", fontsize=8.0,
                    color="#405160", fontweight="bold", fontproperties=title_font, pad=3)
    ax_route = fig.add_subplot(gs[2, :9], projection="3d", computed_zorder=False)
    base.draw_route3d(ax_route, candidates, execution, points, floor_info, legend=False, metrics=True)
    ax_route.set_title(method_titles[4], loc="left", fontsize=8.0,
                       color="#405160", fontweight="bold", fontproperties=title_font, pad=4)
    ax_route.text2D(.03, .075, "J = path length + 0.25 flight time + 2 terminal cost",
                    transform=ax_route.transAxes, fontsize=6.8, color="#72808D")
    ax_profile = fig.add_subplot(gs[2, 9:12]); draw_height_profile(ax_profile, execution, floor_info, compact=True)
    ax_profile.set_title(method_titles[5], loc="left", fontsize=8.0,
                         color="#405160", fontweight="bold", fontproperties=title_font, pad=4)
    rgb_axes = draw_rgb_grid(fig, gs[2, 12:], execution, titles=False)
    rgb_axes[0].set_title(method_titles[6], loc="left", fontsize=7.5,
                          color="#405160", fontweight="bold", fontproperties=title_font, pad=4)
    if not clean_title:
        title = ("方法总览：从语言空间约束到连续三维执行" if locale == "zh" else
                 "Method overview · from spatial language to continuous 3D execution")
        fig.text(.018, .985, title, fontsize=13.2, color="#243442", fontweight="bold",
                 fontproperties=base.FONT_BOLD if locale == "zh" else None, va="top")
    stem = "method_overview_" + ("clean" if clean_title else locale)
    save(fig, stem, tight=False)


def curate():
    SELECTED.mkdir(parents=True, exist_ok=True)
    for old in SELECTED.iterdir():
        if old.is_file(): old.unlink()
    stems = [
        "spatial_relation_microscope", "cross_floor_height_profile",
        "hero_visual_en", "hero_visual_zh", "hero_visual_clean",
        "method_overview_en", "method_overview_zh", "method_overview_clean",
    ]
    for stem in stems:
        for suffix in (".png", ".pdf"):
            shutil.copy2(OUT / f"{stem}{suffix}", SELECTED / f"{stem}{suffix}")
    final_exports = {
        "hero_visual_en": "system_overview_hero",
        "hero_visual_zh": "system_overview_hero_zh",
        "hero_visual_clean": "system_overview_hero_clean",
        "method_overview_en": "system_overview_method",
        "method_overview_zh": "system_overview_method_zh",
        "method_overview_clean": "system_overview_method_clean",
    }
    for stem, target in final_exports.items():
        for suffix in (".png", ".pdf"):
            shutil.copy2(OUT / f"{stem}{suffix}", FINAL / f"{target}{suffix}")
    (SELECTED / "README.md").write_text(
        "# Four-task hero and method figures\n\n"
        "All geometry, trajectory, viewpoint, height, RGB, and detector values come from "
        "`00337_all_found_showcase_v1_20260830`. `hero_visual_en` is the recommended "
        "first-page visual; `method_overview_en` is the recommended method figure. "
        "Clean and Chinese variants share identical data and layout.\n",
        encoding="utf-8",
    )
    manifest = {
        "source_run": str(base.RUN.relative_to(ROOT)),
        "source_map": str(base.PCD.relative_to(ROOT)),
        "source_semantic_map": str(SEMANTIC_MAP.relative_to(ROOT)),
        "source_boxes": str(L2_BOXES.relative_to(ROOT)),
        "representative_relation": {
            "task": MICROWAVE_TASK,
            "anchor": "L2_boxer_30 countertop",
            "target": "L2_boxer_32 microwave",
            "candidate_count": 6,
        },
        "mission": {
            "task_count": 4,
            "feasible_viewpoint_count": 16,
            "executed_path_length_m": 48.106005556437374,
            "elapsed_wall_s": 13.032660698983818,
            "execution_order": base.TASKS,
            "result": "complete; 4/4 found",
        },
        "recommended": {
            "hero": "hero_visual_en",
            "method": "method_overview_en",
        },
    }
    (SELECTED / "figure_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def main():
    language, candidates, execution, points, floor_info = base.load_data()
    render_spatial_microscope(candidates, execution, points)
    render_motion_profile(execution, floor_info)
    for locale in ("en", "zh"):
        render_hero(language, candidates, execution, points, floor_info, locale)
        render_method(language, candidates, execution, points, floor_info, locale)
    render_hero(language, candidates, execution, points, floor_info, "en", clean_title=True)
    render_method(language, candidates, execution, points, floor_info, "en", clean_title=True)
    curate()
    print(f"candidates: {OUT}")
    print(f"selected:   {SELECTED}")


if __name__ == "__main__":
    main()
