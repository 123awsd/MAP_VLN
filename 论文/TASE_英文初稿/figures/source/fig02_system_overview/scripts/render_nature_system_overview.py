#!/usr/bin/env python3
"""Compose an overlapping, scene-led system overview from real 00337 data.

Visual hierarchy:
  official HM3D dollhouse (physical scene)
    ↳ overlapping semantic digital-twin lens (rooms, views, executed route)
       ↳ spatial-grounding microscope (anchor, region, candidate frustums)
          ↳ terminal RGB evidence

No geometric registration between the official website rendering and the
FALCON coordinate frame is implied; the two are linked as physical and mapped
representations of the same HM3D scene.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
from matplotlib.text import Text
from mpl_toolkits.mplot3d import proj3d
from PIL import Image, ImageFilter

import render_four_task_system_overview as base
import render_tro_system_overview as tro

matplotlib.rcParams.update({"pdf.fonttype": 42, "ps.fonttype": 42})


ROOT = base.ROOT
RUN = base.RUN
RAW = ROOT / "paper/system_overview/assets/raw/hm3d_official_00337"
OUT = ROOT / "paper/system_overview/assets/candidates/nature_system_overview"
SELECTED = ROOT / "paper/system_overview/assets/selected/nature_system_overview"
FINAL = ROOT / "paper/system_overview/assets/final"
OFFICIAL = RAW / "dollhouse_highres.png"
OFFICIAL_CUTOUT = OUT / "official_dollhouse_cutout.png"

INK = "#122430"
MUTED = "#61727E"
PAPER = "#FBFCFC"
CYAN = "#1B8E91"
COLORS = base.COLORS
FONT = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")

# Figure-space geometry shared by the evidence renderer and the projected
# callouts.  Keeping this in one place prevents the scientific links from
# drifting away from their real RGB observations during layout iteration.
EVIDENCE_LAYOUT = [
    (.716, .300, .128, .108),
    (.728, .184, .128, .108),
    (.858, .184, .128, .108),
    (.846, .300, .128, .108),
]


def prepare_dollhouse():
    OUT.mkdir(parents=True, exist_ok=True)
    image = Image.open(OFFICIAL).convert("RGBA")
    arr = np.asarray(image).copy()
    rgb = arr[..., :3].astype(float)
    # Official preview background is a flat neutral grey. Preserve scene pixels
    # and soften only the anti-aliased transition around that exact background.
    delta = np.linalg.norm(rgb - np.asarray([128, 128, 128]), axis=2)
    alpha = np.clip((delta - 2.0) / 16.0, 0, 1) * 255
    arr[..., 3] = alpha.astype(np.uint8)
    ys, xs = np.where(alpha > 8)
    pad = 12
    arr = arr[max(0, ys.min()-pad):min(arr.shape[0], ys.max()+pad+1),
              max(0, xs.min()-pad):min(arr.shape[1], xs.max()+pad+1)]
    Image.fromarray(arr).save(OFFICIAL_CUTOUT)


def add_shadowed_image(fig, rect, path, shadow_alpha=.16, shadow_offset=(.008, -.008), zorder=1):
    image = Image.open(path).convert("RGBA")
    alpha = image.getchannel("A").filter(ImageFilter.GaussianBlur(18))
    shadow = Image.new("RGBA", image.size, (18, 32, 40, 0))
    shadow.putalpha(alpha.point(lambda x: int(x * shadow_alpha)))
    ax_shadow = fig.add_axes([rect[0] + shadow_offset[0], rect[1] + shadow_offset[1], rect[2], rect[3]], zorder=zorder)
    ax_shadow.imshow(shadow); ax_shadow.axis("off")
    ax = fig.add_axes(rect, zorder=zorder + 1)
    ax.imshow(image); ax.axis("off")
    return ax


def circle_clip(ax, center=(.5, .5), radius=.49):
    clip = Circle(center, radius, transform=ax.transAxes)
    for artist in ax.get_children():
        if hasattr(artist, "set_clip_path"):
            artist.set_clip_path(clip)


def add_lens_border(fig, rect, zorder):
    overlay = fig.add_axes(rect, zorder=zorder); overlay.axis("off")
    overlay.add_patch(Circle((.5, .5), .49, transform=overlay.transAxes,
                             facecolor="none", edgecolor="white", linewidth=3.1))
    overlay.add_patch(Circle((.5, .5), .49, transform=overlay.transAxes,
                             facecolor="none", edgecolor="#8CA1AA", linewidth=.8, alpha=.9))


def draw_scene_lens(fig, rect, candidates, execution, points, floors, boxes, scene_graph):
    # A subtle halo separates the transparent mapped representation from RGB.
    halo = fig.add_axes([rect[0]-.012, rect[1]-.018, rect[2]+.024, rect[3]+.036], zorder=8)
    halo.axis("off")
    halo.add_patch(Circle((.5, .5), .47, transform=halo.transAxes,
                          facecolor="white", edgecolor="none", alpha=.93))
    ax = fig.add_axes(rect, projection="3d", computed_zorder=False, zorder=9)
    tro.draw_scene(ax, candidates, execution, points, floors, boxes, scene_graph)
    # ``draw_scene`` was also used by a larger standalone figure.  Normalize
    # its text to the physical size of this two-column composition.
    for artist in ax.texts:
        artist.set_fontsize(max(3.6, artist.get_fontsize() * .62))
    ax.dist = 1.95
    ax.patch.set_alpha(0)
    circle_clip(ax)
    add_lens_border(fig, rect, 11)
    return ax


def draw_local_lens(fig, rect, candidates, execution, points, boxes, locale):
    halo = fig.add_axes([rect[0]-.009, rect[1]-.012, rect[2]+.018, rect[3]+.024], zorder=19)
    halo.axis("off")
    halo.add_patch(Circle((.5, .5), .48, transform=halo.transAxes,
                          facecolor="white", edgecolor="none", alpha=.96))
    ax = fig.add_axes(rect, projection="3d", computed_zorder=False, zorder=20)
    tro.draw_spatial_inset(ax, candidates, execution, points, boxes, locale)
    ax.dist = 1.72
    for artist in list(ax.get_children()):
        if isinstance(artist, Text):
            artist.set_visible(False)
    ax.patch.set_alpha(0)
    circle_clip(ax)
    add_lens_border(fig, rect, 22)
    return ax


def connect_projected_target(fig, scene_ax, target_xyz, destination, color):
    # Use Matplotlib's actual 3-D camera projection so the callout originates
    # at the rendered target, not at a hand-estimated image coordinate.
    fig.canvas.draw()
    x2, y2, _ = proj3d.proj_transform(*target_xyz, scene_ax.get_proj())
    display_xy = scene_ax.transData.transform((x2, y2))
    source = fig.transFigure.inverted().transform(display_xy)
    fig.add_artist(Circle(source, .0085, transform=fig.transFigure,
                          facecolor="white", edgecolor=color, linewidth=1.15,
                          zorder=60))
    fig.add_artist(FancyArrowPatch(source, destination, transform=fig.transFigure,
                                   connectionstyle="arc3,rad=-.18", arrowstyle="-",
                                   color=color, linewidth=1.05, alpha=.72, zorder=59))


def draw_quality_inside_lens(fig, rect, candidates, execution, locale):
    ax = fig.add_axes(rect, zorder=24); ax.axis("off")
    group = candidates[base.TASKS[0]]
    chosen = execution["observations"][0]["candidate_id"]
    ordered = sorted(group, key=lambda c: (-c["view_quality"], c["terminal_cost"]))
    xs = np.linspace(.08, .92, len(ordered))
    for x, candidate in zip(xs, ordered):
        selected = candidate["id"] == chosen
        score = len(candidate["visible_region_sample_ids"])
        color = COLORS[base.TASKS[0]] if selected else "#AAB7BE"
        ax.plot([x, x], [.18, .18 + .32 * score / 8], color=color,
                lw=1.8 if selected else .75)
        ax.scatter([x], [.18 + .32 * score / 8], s=26 if selected else 9,
                   color=color, edgecolor="white", lw=.4, zorder=3)
        ax.text(x, .56, f"{score}/8", ha="center", fontsize=3.6, color=color)
    label = "6个候选 · 可见区域覆盖率" if locale == "zh" else "6 candidates · visible-region coverage"
    ax.text(.5, .04, label, ha="center", fontsize=3.7, color=MUTED,
            fontproperties=FONT if locale == "zh" else None)
    ax.set_xlim(0, 1); ax.set_ylim(0, .65)


def draw_instruction(fig, graph, locale):
    ax = fig.add_axes([.045, .865, .63, .105], zorder=30); ax.axis("off")
    instruction = graph["instruction"].replace("朋友快到了，", "")
    # Preserve the original Chinese command while giving it a deliberate
    # editorial line break; relying on the renderer to overflow a single line
    # made the sentence fragile at final paper width.
    instruction = instruction.replace("都还在原位，再顺路", "都还在原位，\n再顺路")
    title = "自然语言任务" if locale == "zh" else "NATURAL-LANGUAGE MISSION"
    ax.text(0, .93, title, color=CYAN, fontsize=5.1, fontweight="bold",
            fontproperties=FONT_BOLD if locale == "zh" else None)
    ax.text(0, .67, instruction, color=INK, fontsize=5.7, fontweight="bold",
            fontproperties=FONT_BOLD, va="top", linespacing=1.35)
    summary = ("Qwen解析出4个独立空间目标；用户未指定访问顺序" if locale == "zh" else
               "Qwen parses four independent spatial goals; the visit order is unspecified")
    ax.text(0, -.10, summary, color=MUTED, fontsize=4.2,
            fontproperties=FONT if locale == "zh" else None)


def draw_task_tokens(fig, locale):
    meta = tro.META_ZH if locale == "zh" else base.META
    positions = [(.055, .795), (.195, .825), (.335, .795), (.475, .825)]
    grounding = [base.TASKS[0], base.TASKS[2], base.TASKS[3], base.TASKS[1]]
    for task_id, (x, y) in zip(grounding, positions):
        label, place, relation = meta[task_id]
        fig.text(x, y, "●", color=COLORS[task_id], fontsize=4.4,
                 fontweight="bold", ha="left", zorder=45)
        fig.text(x+.025, y, label, color=INK, fontsize=5.0, fontweight="bold",
                 fontproperties=FONT_BOLD if locale == "zh" else None, ha="left", zorder=45)
        fig.text(x+.025, y-.025, f"{place} · {relation}", color=MUTED, fontsize=3.7,
                 fontproperties=FONT if locale == "zh" else None, ha="left", zorder=45)


def draw_execution_profile(fig, rect, execution, floors, locale):
    ax = fig.add_axes(rect, zorder=18)
    ax.set_facecolor((1, 1, 1, .90))
    tro.draw_execution_ribbon(ax, execution, floors, locale)
    for artist in ax.texts:
        if artist.get_text().startswith("(e) "):
            artist.set_text("联合三维优化与执行" if locale == "zh" else
                            "Joint 3-D optimization & execution")
            artist.set_fontsize(5.5)
        elif artist.get_text().startswith("$J="):
            artist.set_fontsize(3.4)
            artist.set_position((.85, .92))
        elif "feasible views" in artist.get_text() or "可执行视角" in artist.get_text():
            artist.set_fontsize(3.5)
            artist.set_position((.01, -.20))
        elif artist.get_text()[:2] in {"1 ", "2 ", "3 ", "4 "}:
            artist.set_fontsize(3.55)
            x, y = artist.get_position()
            if artist.get_text().startswith("1 "):
                artist.set_position((x, y + .22))
            elif artist.get_text().startswith("2 "):
                artist.set_position((x, y - .20))
    ax.tick_params(axis="both", labelsize=3.4)
    ax.xaxis.label.set_visible(False)
    ax.yaxis.label.set_fontsize(3.7)
    for spine in ax.spines.values():
        spine.set_linewidth(.55)
    return ax


def draw_detection_crop(ax, observation, locale, target_aspect=2.24):
    """Render a real, undistorted RGB context crop around the OWLv2 box."""
    image = plt.imread(RUN / "execution" / observation["rgb"])
    task_id = observation["task_id"]
    det = observation["verification"]["owlv2"]["detections"][0]
    x1, y1, x2, y2 = np.asarray(det["bbox_xyxy"], float)
    height, width = image.shape[:2]
    box_w, box_h = max(x2 - x1, 1), max(y2 - y1, 1)
    crop_w = min(width, max(330.0, box_w * 3.2, box_h * target_aspect * 2.4))
    crop_h = min(height, crop_w / target_aspect)
    crop_w = min(width, crop_h * target_aspect)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    left = np.clip(cx - crop_w / 2, 0, width - crop_w)
    top = np.clip(cy - crop_h / 2, 0, height - crop_h)
    right, bottom = left + crop_w, top + crop_h
    cropped = image[int(round(top)):int(round(bottom)), int(round(left)):int(round(right))]
    ax.imshow(cropped)
    ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Rectangle((x1 - left, y1 - top), box_w, box_h, fill=False,
                           edgecolor=COLORS[task_id], linewidth=1.45))
    target_name = (tro.META_ZH if locale == "zh" else base.META)[task_id][0]
    ax.text(.025, .045, f"{target_name}  {det['score']:.2f}", transform=ax.transAxes,
            color="white", fontsize=4.25, fontweight="bold", va="bottom", ha="left",
            bbox=dict(fc=COLORS[task_id], ec="none", pad=1.0),
            fontproperties=FONT_BOLD if locale == "zh" else None)
    seq = int(observation["sequence"]) + 1
    ax.text(.025, .94, str(seq), transform=ax.transAxes, va="top", color="white",
            fontsize=4.8, fontweight="bold",
            bbox=dict(boxstyle="circle,pad=.18", fc=COLORS[task_id], ec="white", lw=.55))
    for spine in ax.spines.values():
        spine.set_color(COLORS[task_id]); spine.set_linewidth(1.0)


def draw_rgb_evidence(fig, execution, locale):
    # A staggered photographic sequence overlaps the mapped scene.  A faint
    # offset shadow gives the RGB frames physical depth without turning them
    # into dashboard cards.
    for i, (observation, (x, y, w, h)) in enumerate(zip(execution["observations"], EVIDENCE_LAYOUT)):
        fig.add_artist(Rectangle((x + .004, y - .005), w, h,
                                 transform=fig.transFigure, facecolor="#203540",
                                 edgecolor="none", alpha=.10, zorder=29+i))
        ax = fig.add_axes([x, y, w, h], zorder=31+i)
        draw_detection_crop(ax, observation, locale)
        for spine in ax.spines.values():
            spine.set_linewidth(1.15)
    label = "在线视觉证据 · 4/4确认" if locale == "zh" else "ONLINE EVIDENCE · 4/4 CONFIRMED"
    fig.text(.73, .415, label, fontsize=5.0, color=INK, fontweight="bold",
             fontproperties=FONT_BOLD if locale == "zh" else None, zorder=40)


def draw_annotations(fig, locale):
    digital = "三维语义数字孪生" if locale == "zh" else "3-D SEMANTIC DIGITAL TWIN"
    local = "空间约束观察位姿" if locale == "zh" else "SPATIALLY GROUNDED VIEWS"
    fig.text(.505, .735, digital, color=INK, fontsize=5.5, fontweight="bold",
             fontproperties=FONT_BOLD if locale == "zh" else None, zorder=50)
    digital_detail = ("房间 · 三维OBB · 可执行/选中视角 · 已执行B样条" if locale == "zh" else
                      "rooms · 3-D OBBs · feasible/selected views · executed B-spline")
    fig.text(.505, .712, digital_detail, color=MUTED, fontsize=3.8, zorder=50,
             fontproperties=FONT if locale == "zh" else None)
    fig.text(.745, .75, local, color=INK, fontsize=5.0, fontweight="bold",
             fontproperties=FONT_BOLD if locale == "zh" else None, zorder=50)
    detail = ("靠近(台面) → 搜索区域 → FREE∩碰撞∩FoV∩视线" if locale == "zh" else
              "near(countertop) → region → FREE ∩ collision ∩ FoV ∩ LoS")
    fig.text(.745, .728, detail, color=MUTED, fontsize=3.5,
             fontproperties=FONT if locale == "zh" else None, zorder=50)


def draw_source_note(fig, locale):
    note = ("真实HM3D 00337场景 · 官方完整场景视图 · 实际Stage1/Stage2数据仅显示并执行L1–L3" if locale == "zh" else
            "Real HM3D 00337 · official full-scene context · recorded Stage-1/Stage-2 map and execution show L1–L3")
    fig.text(.046, .022, note, color="#72818B", fontsize=3.45,
             fontproperties=FONT if locale == "zh" else None)


def render(locale="en"):
    prepare_dollhouse()
    graph, candidates, execution, points, floors = base.load_data()
    boxes = tro.load_box_tables()
    scene_graph = tro.read_json(tro.SCENE_GRAPH)

    # The physical size is deliberately close to the final two-column paper
    # width.  This keeps 5--10 pt annotations legible after LaTeX placement;
    # the previous 17.2-inch canvas looked fine on screen but shrank text too
    # aggressively in a paper.
    fig = plt.figure(figsize=(7.16, 3.78), facecolor=PAPER)
    add_shadowed_image(fig, [.015, .045, .69, .79], OFFICIAL_CUTOUT,
                       shadow_alpha=.14, shadow_offset=(.008, -.012), zorder=1)

    draw_instruction(fig, graph, locale)
    draw_task_tokens(fig, locale)
    scene_ax = draw_scene_lens(fig, [.405, .13, .36, .68], candidates, execution, points, floors, boxes, scene_graph)
    draw_local_lens(fig, [.69, .50, .19, .36], candidates, execution, points, boxes, locale)
    draw_quality_inside_lens(fig, [.72, .535, .13, .10], candidates, execution, locale)
    draw_execution_profile(fig, [.46, .055, .40, .155], execution, floors, locale)
    draw_rgb_evidence(fig, execution, locale)
    draw_annotations(fig, locale)
    draw_source_note(fig, locale)
    target = np.asarray(candidates[base.TASKS[0]][0]["target_xyz_m"], float)
    connect_projected_target(fig, scene_ax, target, (.702, .635), COLORS[base.TASKS[0]])

    OUT.mkdir(parents=True, exist_ok=True)
    stem = f"nature_system_overview_{locale}"
    fig.savefig(OUT / f"{stem}.png", dpi=360, facecolor=PAPER)
    fig.savefig(OUT / f"{stem}.pdf", facecolor=PAPER)
    plt.close(fig)
    return stem


def curate(stems):
    SELECTED.mkdir(parents=True, exist_ok=True); FINAL.mkdir(parents=True, exist_ok=True)
    for stem in stems:
        for suffix in (".png", ".pdf"):
            shutil.copy2(OUT / f"{stem}{suffix}", SELECTED / f"{stem}{suffix}")
    for suffix in (".png", ".pdf"):
        shutil.copy2(OUT / f"nature_system_overview_en{suffix}",
                     FINAL / f"system_overview_nature{suffix}")
        shutil.copy2(OUT / f"nature_system_overview_zh{suffix}",
                     FINAL / f"system_overview_nature_zh{suffix}")
    manifest = {
        "official_scene_page": "https://aihabitat.org/datasets/hm3d/00337-CFVBbU9Rsyb/index.html",
        "official_dollhouse": "https://habitatwebsite.s3.amazonaws.com/website-visualization/00337-CFVBbU9Rsyb/dollhouse_highres.png",
        "source_run": str(RUN.relative_to(ROOT)),
        "registration_note": "The official dollhouse and FALCON map are linked representations of the same scene; no pixel-level registration is claimed.",
        "paper_size_inches": [7.16, 3.78],
        "scene_scope": "Official dollhouse shows full-scene context; recorded map and execution show L1-L3.",
        "facts": {"tasks": 4, "feasible_views": 16, "path_m": 48.1060055564,
                  "found": 4, "active_recovery_executed": False},
    }
    (SELECTED / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8")


def main():
    stems = [render("en"), render("zh")]
    curate(stems)
    print(FINAL / "system_overview_nature.png")


if __name__ == "__main__":
    main()
