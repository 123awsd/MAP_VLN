#!/usr/bin/env python3
"""Create deterministic paper-figure variants for one candidate-viewpoint task."""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, Polygon
from PIL import Image, ImageDraw, ImageFont


COLORS = {
    "candidate": "#4D9DE0",
    "retry": "#F59E0B",
    "found": "#16A36A",
    "target": "#E53935",
    "region": "#F2C94C",
    "structure": "#A98B6D",
    "ink": "#25313C",
}


def rotated_rectangle(center, size, yaw, scale=1.0):
    hx, hy = 0.5 * np.asarray(size[:2], dtype=float) * scale
    local = np.asarray([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    return local @ rotation.T + np.asarray(center[:2], dtype=float)


def camera_glyph(ax, xy, yaw, color, scale=0.13, zorder=8, linewidth=1.8):
    forward = np.asarray([math.cos(yaw), math.sin(yaw)])
    left = np.asarray([-forward[1], forward[0]])
    tip = xy + scale * 1.45 * forward
    rear = xy - scale * 0.55 * forward
    triangle = np.asarray([tip, rear + scale * 0.72 * left, rear - scale * 0.72 * left])
    ax.add_patch(Polygon(triangle, closed=True, facecolor="white", edgecolor=color,
                         linewidth=linewidth, zorder=zorder))
    ax.add_patch(Circle(xy, scale * 0.25, facecolor=color, edgecolor="white",
                        linewidth=0.7, zorder=zorder + 1))


def status_map(execution, task_id):
    result = {}
    for item in execution.get("observations", []):
        if item.get("task_id") == task_id:
            result[item.get("candidate_id")] = item.get("outcome")
    return result


def draw_topdown(ax, candidates, table, backpack_xy, outcomes, compact=False):
    center = np.asarray(table["center_xyz_m"][:2], dtype=float)
    size = np.asarray(table["size_xyz_m"][:2], dtype=float)
    w, x, y, z = map(float, table.get("orientation_wxyz", [1, 0, 0, 0]))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    ax.set_facecolor("#FBFAF7")
    all_xy = np.asarray([[c["pose"]["x"], c["pose"]["y"]] for c in candidates])
    radii = sorted({round(float(np.linalg.norm(point - center)), 3) for point in all_xy})
    for radius in radii:
        ax.add_patch(Circle(center, radius, fill=False, edgecolor="#A8B3BD",
                            linestyle=(0, (4, 4)), linewidth=1.1, alpha=0.75, zorder=1))

    search = rotated_rectangle(center, size, yaw, scale=0.80)
    ax.add_patch(Polygon(search, closed=True, facecolor=COLORS["region"],
                         edgecolor="#C58A00", linewidth=1.5, alpha=0.28,
                         hatch="///", zorder=3))
    footprint = rotated_rectangle(center, size, yaw)
    ax.add_patch(Polygon(footprint, closed=True, facecolor=COLORS["structure"],
                         edgecolor="#735C48", linewidth=2.3, alpha=0.38, zorder=4))
    for corner in footprint:
        ax.add_patch(Circle(corner, 0.065, facecolor="#735C48", edgecolor="white",
                            linewidth=0.8, zorder=5))

    ax.scatter([backpack_xy[0]], [backpack_xy[1]], s=165, marker="s",
               color=COLORS["target"], edgecolors="white", linewidths=1.7,
               zorder=9)
    if not compact:
        ax.annotate("backpack", backpack_xy, xytext=(9, -18), textcoords="offset points",
                    fontsize=10, color=COLORS["target"], fontweight="bold")
        ax.annotate("below-table search region", center, xytext=(0, 12),
                    textcoords="offset points", ha="center", fontsize=9,
                    color="#6E5700", fontweight="bold", zorder=10)

    for candidate in candidates:
        pose = candidate["pose"]
        xy = np.asarray([pose["x"], pose["y"]], dtype=float)
        outcome = outcomes.get(candidate["id"])
        color = COLORS["found"] if outcome == "found" else (
            COLORS["retry"] if outcome == "retry" else COLORS["candidate"]
        )
        ax.plot([xy[0], center[0]], [xy[1], center[1]], color=color,
                linewidth=0.85, alpha=0.35, zorder=2)
        camera_glyph(ax, xy, float(pose["yaw"]), color,
                     scale=0.15 if outcome == "found" else 0.12,
                     linewidth=2.8 if outcome == "found" else 1.7)

    pad = 0.42
    mn = np.minimum(all_xy.min(axis=0), backpack_xy) - pad
    mx = np.maximum(all_xy.max(axis=0), backpack_xy) + pad
    ax.set_xlim(mn[0], mx[0]); ax.set_ylim(mn[1], mx[1])
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def make_topdown(candidates, table, backpack_xy, outcomes, output):
    fig, ax = plt.subplots(figsize=(8, 5.5), dpi=200)
    draw_topdown(ax, candidates, table, backpack_xy, outcomes)
    ax.set_title("Candidate Viewpoint Generation", fontsize=17, fontweight="bold",
                 color=COLORS["ink"], pad=12)
    handles = [
        plt.Line2D([0], [0], marker="^", color="none", markeredgecolor=COLORS["candidate"],
                   markerfacecolor="white", markersize=9, label="Feasible candidate"),
        plt.Line2D([0], [0], marker="^", color="none", markeredgecolor=COLORS["retry"],
                   markerfacecolor="white", markersize=9, label="Retried viewpoint"),
        plt.Line2D([0], [0], marker="^", color="none", markeredgecolor=COLORS["found"],
                   markerfacecolor="white", markersize=9, label="Verified viewpoint"),
    ]
    ax.legend(handles=handles, loc="upper left", frameon=False, fontsize=9, ncol=3)
    fig.tight_layout(pad=1.2)
    fig.savefig(output, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def make_side(table, candidate_z, floor_z, display_offset, output):
    top = float(table["center_xyz_m"][2] + table["size_xyz_m"][2] / 2)
    bottom = float(table["center_xyz_m"][2] - table["size_xyz_m"][2] / 2)
    shown_z = candidate_z + display_offset
    fig, ax = plt.subplots(figsize=(8, 5.5), dpi=200)
    ax.set_facecolor("#FBFAF7")
    ax.plot([-2.5, 2.5], [floor_z, floor_z], color="#7B8791", linewidth=3)
    ax.add_patch(plt.Rectangle((-1.45, top - 0.08), 2.9, 0.12,
                               facecolor=COLORS["structure"], edgecolor="#735C48", lw=2))
    for x in (-1.25, 1.25):
        ax.add_patch(plt.Rectangle((x - 0.055, bottom), 0.11, top - bottom,
                                   facecolor=COLORS["structure"], edgecolor="#735C48", lw=1.5))
    ax.add_patch(plt.Rectangle((-1.18, bottom + 0.02), 2.36, 0.17,
                               facecolor=COLORS["region"], edgecolor="#C58A00",
                               alpha=0.28, hatch="///"))
    ax.scatter([0.70], [floor_z + 0.18], marker="s", s=180,
               color=COLORS["target"], edgecolor="white", linewidth=1.5, zorder=8)
    for x in (-2.0, -1.65, 1.65, 2.0):
        direction = 1 if x < 0 else -1
        ax.scatter([x], [shown_z], marker=">" if direction > 0 else "<", s=130,
                   facecolor="white", edgecolor=COLORS["candidate"], linewidth=2, zorder=7)
        ax.plot([x, 0], [shown_z, bottom + 0.10], color=COLORS["candidate"], alpha=0.38, lw=1)
    ax.annotate("below-region", (0, bottom + 0.10), ha="center", va="center",
                fontsize=10, color="#6E5700", fontweight="bold")
    ax.annotate("backpack", (0.70, floor_z + 0.18), xytext=(18, -2),
                textcoords="offset points", color=COLORS["target"], fontweight="bold")
    ax.annotate("candidate camera height", (-2.0, shown_z), xytext=(0, 24),
                textcoords="offset points", ha="left", fontsize=9, color=COLORS["ink"])
    ax.set_xlim(-2.65, 2.65); ax.set_ylim(floor_z - 0.12, top + 0.42)
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values(): spine.set_visible(False)
    ax.set_title("Side-view Relation Schematic", fontsize=17, fontweight="bold",
                 color=COLORS["ink"], pad=12)
    fig.tight_layout(pad=1.2)
    fig.savefig(output, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def make_hybrid(topdown, rgb, output):
    left = Image.open(topdown).convert("RGB")
    right = Image.open(rgb).convert("RGB")
    canvas = Image.new("RGB", (1800, 900), "white")
    left.thumbnail((1050, 850), Image.Resampling.LANCZOS)
    right.thumbnail((700, 760), Image.Resampling.LANCZOS)
    canvas.paste(left, ((1080 - left.width) // 2, (900 - left.height) // 2))
    canvas.paste(right, (1080, 82))
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle((1068, 70, 1792, 854), radius=16, outline="#D2D8DD", width=4)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    font = ImageFont.truetype(font_path, 25)
    draw.text((1100, 28), "Visual verification at selected viewpoint",
              fill="#25313C", font=font)
    draw.line((1020, 450, 1070, 450), fill="#16A36A", width=7)
    draw.polygon([(1068, 450), (1048, 438), (1048, 462)], fill="#16A36A")
    small = ImageFont.truetype(font_path, 21)
    steps = ["Language relation", "Search region", "Pose sampling", "Geometric filtering"]
    y0 = 665
    for index, label in enumerate(steps):
        x0 = 1100 + (index % 2) * 335
        y = y0 + (index // 2) * 82
        draw.rounded_rectangle((x0, y, x0 + 285, y + 48), radius=12,
                               fill="#F4F7F9", outline="#BFC9D1", width=2)
        draw.text((x0 + 18, y + 10), label, fill="#25313C", font=small)
    canvas.save(output)


def make_contact_sheet(paths, output):
    canvas = Image.new("RGB", (1800, 1220), "#EEF1F3")
    draw = ImageDraw.Draw(canvas)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    title_font = ImageFont.truetype(font_path, 30)
    labels = [
        "A  RGB overlay", "B  Top-down schematic",
        "C  Side-view schematic", "D  Hybrid method figure",
    ]
    for index, (path, label) in enumerate(zip(paths, labels)):
        image = Image.open(path).convert("RGB")
        image.thumbnail((840, 500), Image.Resampling.LANCZOS)
        col, row = index % 2, index // 2
        x = 40 + col * 880
        y = 55 + row * 590
        draw.text((x, y - 42), label, fill="#25313C", font=title_font)
        panel = Image.new("RGB", (840, 500), "white")
        panel.paste(image, ((840 - image.width) // 2, (500 - image.height) // 2))
        canvas.paste(panel, (x, y))
        draw.rectangle((x, y, x + 840, y + 500), outline="#CBD2D8", width=3)
    canvas.save(output)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--candidates", type=Path, required=True)
    ap.add_argument("--execution", type=Path, required=True)
    ap.add_argument("--scene-graph", type=Path, required=True)
    ap.add_argument("--rgb-overlay", type=Path, required=True)
    ap.add_argument("--rgb-detection", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--task", default="find_backpack_under_table_l2")
    ap.add_argument("--runtime-object", type=Path, required=True)
    ap.add_argument("--display-z-offset", type=float, default=-0.20)
    args = ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    pool = json.loads(args.candidates.read_text())["by_task"][args.task]
    scene = json.loads(args.scene_graph.read_text())
    objects = {item["id"]: item for item in scene["objects"]}
    table = objects[pool[0]["object_id"]]
    execution = json.loads(args.execution.read_text())
    outcomes = status_map(execution, args.task)
    runtime = json.loads(args.runtime_object.read_text())["objects"][0]
    backpack_xy = np.asarray(runtime["position_xyz_f"][:2], dtype=float)
    floor = next(item for item in scene["floors"] if item["floor_id"] == pool[0]["floor_id"])

    rgb_out = args.output_dir / "方案A_真实RGB叠加.png"
    top_out = args.output_dir / "方案B_俯视学术示意.png"
    side_out = args.output_dir / "方案C_侧视空间关系示意.png"
    hybrid_out = args.output_dir / "方案D_示意加真实检测.png"
    contact_out = args.output_dir / "四种表示方式_对比预览.png"
    shutil.copy2(args.rgb_overlay, rgb_out)
    make_topdown(pool, table, backpack_xy, outcomes, top_out)
    make_side(table, float(pool[0]["pose"]["z"]), float(floor["floor_z_m"]),
              args.display_z_offset, side_out)
    make_hybrid(top_out, args.rgb_detection, hybrid_out)
    make_contact_sheet((rgb_out, top_out, side_out, hybrid_out), contact_out)

    manifest = {
        "format": "pre_map_vln.candidate_viewpoint_figure_variants.v1",
        "task_id": args.task,
        "source_candidate_count": len(pool),
        "source_candidate_z_m": sorted({item["pose"]["z"] for item in pool}),
        "display_z_offset_m": args.display_z_offset,
        "data_note": "Only the visualization is offset; source candidates are unchanged.",
        "outputs": [str(p.resolve()) for p in
                    (rgb_out, top_out, side_out, hybrid_out, contact_out)],
    }
    (args.output_dir / "绘图数据说明.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    for path in (rgb_out, top_out, side_out, hybrid_out, contact_out):
        print(path)


if __name__ == "__main__":
    main()
