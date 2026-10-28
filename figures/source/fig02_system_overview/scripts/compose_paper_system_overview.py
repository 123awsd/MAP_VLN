#!/usr/bin/env python3
"""Compose a dense paper-facing System Overview from real project outputs."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from PIL import Image


ROOT = Path(__file__).resolve().parents[3]
SELECTED = ROOT / "paper/system_overview/assets/selected"
FINAL = ROOT / "paper/system_overview/assets/final"

NAVY = "#17324d"
GRAY = "#65758b"
GREEN = "#3a7d44"
BLUE = "#2f6f9f"
PURPLE = "#7b3f86"
ORANGE = "#c56522"
RED = "#b83a45"


def load(path: Path, crop=None):
    image = Image.open(path).convert("RGB")
    if crop is not None:
        w, h = image.size
        left, top, right, bottom = crop
        image = image.crop((int(left * w), int(top * h), int(right * w), int(bottom * h)))
    return image


def panel(ax, xywh, title, edge, face):
    x, y, w, h = xywh
    patch = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=.008",
                           linewidth=2.0, linestyle=(0, (5, 3)),
                           edgecolor=edge, facecolor=face, zorder=0)
    ax.add_patch(patch)
    ax.text(x + .015, y + h - .028, title, ha="left", va="top",
            fontsize=13, weight="bold", color=edge)


def image_axis(fig, box, image, title=None, border=None):
    ax = fig.add_axes(box)
    ax.imshow(image)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(border is not None)
        if border:
            spine.set_color(border)
            spine.set_linewidth(2)
    if title:
        ax.set_title(title, fontsize=9, pad=3, weight="bold", color=NAVY)
    return ax


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=FINAL / "system_overview")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(18, 9.4), facecolor="white")
    canvas = fig.add_axes([0, 0, 1, 1])
    canvas.set_xlim(0, 1)
    canvas.set_ylim(0, 1)
    canvas.axis("off")

    # Header: a concrete, natural multi-task instruction.
    canvas.text(.5, .965, "PRE-MAP-VLN: Language-Guided Long-Horizon Task Execution with a Reusable 3D Semantic Map",
                ha="center", va="top", fontsize=18, weight="bold", color=NAVY)
    instruction = ("“Check the microwave on the kitchen counter, the coffee table beside the sofa, "
                   "the bag under the bed, and the TV in the living room; arrange the route yourself.”")
    canvas.add_patch(FancyBboxPatch((.055, .865), .89, .062, boxstyle="round,pad=.008",
                                    linewidth=1.5, edgecolor=GRAY, facecolor="#f8fafc"))
    canvas.text(.5, .896, instruction, ha="center", va="center", fontsize=11, color=NAVY)

    # Three substantive modules; every module contains real project images.
    panel(canvas, (.025, .08, .31, .74), "A  Pre-built 3D semantic map (offline, floors 1--3)", GREEN, "#f3f8f1")
    panel(canvas, (.355, .08, .29, .74), "B  Language-to-executable mission planning", PURPLE, "#f7f2f8")
    panel(canvas, (.665, .08, .31, .74), "C  Online flight, verification, and update", RED, "#fcf3f3")

    # A: real 3D explored map and compact floor semantics.
    explored = load(SELECTED / "stage1/00337_truth_vs_falcon_3d.png", crop=(.48, .08, 1, .52))
    rooms = load(SELECTED / "room_pipeline/00337_room_semantics.png")
    image_axis(fig, [.045, .445, .27, .30], explored, "FALCON 3D occupancy + executed exploration trajectory")
    image_axis(fig, [.045, .14, .27, .23], rooms, "Transition-aware room regions and Qwen room semantics")
    canvas.text(.18, .105, "occupancy  +  object boxes  +  rooms  +  inter-floor transitions",
                ha="center", va="center", fontsize=9.5, color=GREEN, weight="bold")

    # B1: structured task graph, drawn compactly to avoid crossing edges.
    graph_y = .655
    nodes = [(.395, "microwave\nL2 kitchen"), (.485, "coffee table\nL2 living"),
             (.575, "bag\nunder bed L1"), (.485, "TV\nL3 living")]
    for x, text in nodes:
        y = graph_y if "TV" not in text else .555
        canvas.add_patch(FancyBboxPatch((x - .037, y - .034), .074, .068,
                                        boxstyle="round,pad=.006", linewidth=1.3,
                                        edgecolor=PURPLE, facecolor="white"))
        canvas.text(x, y, text, ha="center", va="center", fontsize=7.3, color=NAVY, weight="bold")
    canvas.text(.5, .755, "Qwen task graph", ha="center", fontsize=10.5, weight="bold", color=PURPLE)
    canvas.text(.5, .715, "spatial constraints preserved  •  no mandatory order in this instruction",
                ha="center", fontsize=8.5, color=GRAY)
    canvas.text(.5, .515, "joint optimizer chooses task order and one executable pose per task",
                ha="center", fontsize=8.2, color=PURPLE, style="italic")

    # B2: actual candidate view geometry.
    candidate = load(SELECTED / "generated/候选观察位姿_三维视锥.png", crop=(.05, .10, .97, .94))
    image_axis(fig, [.38, .17, .24, .32], candidate, "Task-conditioned 3D observation poses")
    canvas.text(.5, .115, "shared 3D occupancy semantics  →  coarse 3D A* cost  →  full A* + B-spline validation",
                ha="center", fontsize=8.5, color=PURPLE, weight="bold")

    # C: actual trajectory plus terminal visual evidence.
    execution = load(SELECTED / "stage2/00337_stage2_task_execution_summary.png", crop=(0, 0, .59, 1))
    image_axis(fig, [.688, .39, .265, .35], execution, "Continuous multi-floor UAV execution")
    evidence = [
        ("microwave", load(SELECTED / "stage2/00337_terminal_microwave.jpg")),
        ("coffee table", load(SELECTED / "stage2/00337_terminal_coffee_table.jpg")),
        ("television", load(SELECTED / "stage2/00337_terminal_tv.jpg")),
    ]
    positions = [.69, .785, .88]
    for x, (label, image) in zip(positions, evidence):
        image_axis(fig, [x, .18, .082, .14], image, label, border="#35a763")
    canvas.text(.82, .125, "OWLv2 observation  →  RGB-D projection  →  map update / task outcome",
                ha="center", fontsize=9, color=RED, weight="bold")

    # Main flow and online feedback loop.
    for x1, x2 in ((.337, .353), (.647, .663)):
        canvas.add_patch(FancyArrowPatch((x1, .47), (x2, .47), arrowstyle="simple",
                                         mutation_scale=22, color=ORANGE, linewidth=0))
    canvas.add_patch(FancyArrowPatch((.91, .09), (.50, .09), arrowstyle="-|>",
                                     mutation_scale=13, linewidth=1.8,
                                     linestyle="--", color=RED,
                                     connectionstyle="arc3,rad=-.15"))
    canvas.text(.72, .045, "failed verification → conditional replanning / semantic recovery",
                ha="center", fontsize=9, color=RED, weight="bold")

    for ext in ("png", "pdf"):
        fig.savefig(args.output.with_suffix(f".{ext}"), dpi=240, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"generated {args.output}.png and {args.output}.pdf")


if __name__ == "__main__":
    main()
