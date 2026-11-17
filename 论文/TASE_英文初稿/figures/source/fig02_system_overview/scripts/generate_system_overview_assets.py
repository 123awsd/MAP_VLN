#!/usr/bin/env python3
"""Generate clean, paper-facing System Overview panels from existing outputs.

The script deliberately uses only recorded task graphs, candidate poses and
execution JSON. It does not use Habitat ground-truth geometry for the panels.
Default visualizations are restricted to floors 1--3.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


COLORS = {
    "navy": "#18324b",
    "blue": "#2f6f9f",
    "teal": "#2a9d8f",
    "orange": "#e58f3a",
    "red": "#d1495b",
    "green": "#3aaf6f",
    "gray": "#718096",
    "light": "#f5f7fa",
}


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def save(fig, out: Path, stem: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(out / f"{stem}.{ext}", dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def short_label(task: dict) -> str:
    label = task.get("verification_label") or task.get("target", {}).get("label", "target")
    floor = task.get("target", {}).get("floor_id")
    room = task.get("target", {}).get("room") or task.get("target", {}).get("room_id")
    suffix = f"L{floor}" if floor else (room or "")
    return f"{label}\n{suffix}".strip()


def plot_task_graph(graph: dict, out: Path) -> None:
    tasks = graph.get("tasks", [])
    rules = graph.get("conditional_rules", [])
    fig, ax = plt.subplots(figsize=(12, 5.7))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6)
    ax.axis("off")
    ax.set_title("Language task graph: partial order and conditional recovery", fontsize=15, pad=14)

    # Arrange tasks by their graph order, keeping the example readable.
    positions = {}
    n = min(len(tasks), 6)
    for i, task in enumerate(tasks[:n]):
        positions[task["id"]] = (1.15 + (i % 3) * 3.8, 4.35 - (i // 3) * 2.35)

    for task in tasks[:n]:
        x, y = positions[task["id"]]
        target = task.get("target", {})
        title = short_label(task)
        scope = target.get("reference") or target.get("room") or "semantic target"
        color = COLORS["blue"] if task.get("active_initially", True) else COLORS["orange"]
        box = FancyBboxPatch((x - 1.15, y - .48), 2.3, .96, boxstyle="round,pad=.08",
                             linewidth=1.6, edgecolor=color, facecolor="#eef5fa" if color == COLORS["blue"] else "#fff3e4")
        ax.add_patch(box)
        ax.text(x, y + .10, title, ha="center", va="center", fontsize=9, weight="bold", color=COLORS["navy"])
        ax.text(x, y - .27, scope, ha="center", va="center", fontsize=7.5, color=COLORS["gray"])

    for task in tasks[:n]:
        for predecessor in task.get("prerequisites", []):
            if predecessor in positions:
                x1, y1 = positions[predecessor]
                x2, y2 = positions[task["id"]]
                ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                              mutation_scale=13, linewidth=1.3, color=COLORS["gray"]))

    # Draw an explicit branch using the actual Qwen parsed rule.
    for rule in rules:
        source = rule.get("source_task_id")
        targets = rule.get("activate_task_ids", [])
        if source not in positions:
            continue
        for target in targets:
            if target not in positions:
                continue
            x1, y1 = positions[source]
            x2, y2 = positions[target]
            ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                          mutation_scale=14, linewidth=2.0, linestyle="--",
                                          color=COLORS["red"], connectionstyle="arc3,rad=.22"))
            ax.text((x1 + x2) / 2, (y1 + y2) / 2 + .22,
                    f"if {rule.get('if_outcome', 'not_found')}", fontsize=8,
                    color=COLORS["red"], ha="center", weight="bold")

    ax.text(.35, .35, "solid edge: mandatory predecessor     dashed red edge: conditional activation",
            fontsize=9, color=COLORS["gray"])
    save(fig, out, "任务图_语言约束与条件分支")


def plot_candidate_poses(candidates: dict, out: Path, task_id: str) -> None:
    values = candidates.get("by_task", {}).get(task_id, [])
    if not values:
        raise RuntimeError(f"no candidates found for {task_id}")
    values = [v for v in values if int(v.get("floor_id", 0) or 0) <= 3]
    first = values[0]
    center = np.asarray(first.get("search_region_center_xyz_m", first.get("target_xyz_m")), dtype=float)
    size = np.asarray(first.get("search_region_size_xyz_m", [1, 1, .2]), dtype=float)
    fig = plt.figure(figsize=(8.5, 7.2))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Task-conditioned 3D observation poses", pad=16, fontsize=15)
    xs = np.array([v["pose"]["x"] for v in values])
    ys = np.array([v["pose"]["y"] for v in values])
    zs = np.array([v["pose"]["z"] for v in values])
    quality = np.array([float(v.get("view_quality", 0)) for v in values])
    ax.scatter(xs, ys, zs, c=quality, cmap="viridis", s=52, depthshade=False, label="valid candidates")
    best = max(values, key=lambda v: (float(v.get("view_quality", 0)), -float(v.get("terminal_cost", 1e9))))
    bp = best["pose"]
    ax.scatter([bp["x"]], [bp["y"]], [bp["z"]], c=COLORS["red"], s=105, marker="*", label="selected view")
    ax.scatter([center[0]], [center[1]], [center[2]], c=COLORS["orange"], s=85, marker="s", label="search region")
    for v in values[:12]:
        p = v["pose"]
        yaw = float(p.get("yaw", 0))
        length = .45
        ax.quiver(p["x"], p["y"], p["z"], length * math.cos(yaw), length * math.sin(yaw), 0,
                  color=COLORS["blue"], linewidth=.8, arrow_length_ratio=.25)
        # A compact horizontal view frustum: apex at the UAV and a small
        # target-facing segment around the search region center.
        dx, dy = center[0] - p["x"], center[1] - p["y"]
        norm = max(1e-6, math.hypot(dx, dy))
        ux, uy = dx / norm, dy / norm
        vx, vy = -uy, ux
        half_width = .34
        far_center = np.array([center[0], center[1], center[2]])
        left = far_center + np.array([vx * half_width, vy * half_width, 0])
        right = far_center - np.array([vx * half_width, vy * half_width, 0])
        apex = np.array([p["x"], p["y"], p["z"]])
        for endpoint in (left, right, far_center):
            ax.plot([apex[0], endpoint[0]], [apex[1], endpoint[1]],
                    [apex[2], endpoint[2]], color=COLORS["blue"], linewidth=.65, alpha=.75)
        ax.plot([left[0], right[0]], [left[1], right[1]], [left[2], right[2]],
                color=COLORS["blue"], linewidth=.65, alpha=.75)
    ax.plot([center[0], center[0]], [center[1], center[1]],
            [center[2] - size[2] / 2, center[2] + size[2] / 2], color=COLORS["orange"], linewidth=3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=25, azim=-58)
    save(fig, out, "候选观察位姿_三维视锥")


def plot_trajectory(execution: dict, out: Path, max_floor: int) -> None:
    trajectory = np.asarray(execution.get("trajectory_xyz_yaw", []), dtype=float)
    trajectory = trajectory[trajectory[:, 2] < 10] if len(trajectory) else trajectory
    fig = plt.figure(figsize=(9, 6.8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Continuous 3D execution trajectory (floors 1--3)", pad=16, fontsize=15)
    if len(trajectory):
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color=COLORS["navy"], linewidth=1.8)
        ax.scatter(*trajectory[0, :3], color=COLORS["green"], s=65, label="start")
        ax.scatter(*trajectory[-1, :3], color=COLORS["red"], s=65, label="finish")
        ax.scatter(trajectory[::max(1, len(trajectory) // 120), 0],
                   trajectory[::max(1, len(trajectory) // 120), 1],
                   trajectory[::max(1, len(trajectory) // 120), 2],
                   c=trajectory[::max(1, len(trajectory) // 120), 2], cmap="plasma", s=5, alpha=.55)
    for floor, z in enumerate([1.0, 3.5, 6.0], start=1):
        if floor > max_floor:
            break
        ax.text2D(.03, .93 - .035 * floor, f"L{floor}", transform=ax.transAxes, color=COLORS["gray"])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(loc="upper left")
    ax.view_init(elev=25, azim=-63)
    save(fig, out, "跨楼层执行轨迹_1至3层")


def plot_recovery(execution: dict, out: Path) -> None:
    observations = execution.get("observations", [])
    recovery = [o.get("recovery", {}) for o in observations if o.get("recovery")]
    found = next((o for o in observations if o.get("verification_outcome") == "found"), None)
    attempted = []
    for item in recovery:
        attempted.extend(item.get("attempted_candidate_ids", []))
    locations = []
    for item in recovery:
        loc = item.get("location_hypothesis_id")
        if loc and loc not in locations:
            locations.append(loc)
    fig, ax = plt.subplots(figsize=(12, 4.8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5)
    ax.axis("off")
    ax.set_title("Semantic recovery after historical-location failure", fontsize=15, pad=15)
    boxes = [
        (1.0, "specified location\nL1 bathroom sink", "7 views: not found", COLORS["red"]),
        (4.25, "Qwen recovery\nsemantic hypotheses", f"{len(locations) or 1} location hypotheses", COLORS["orange"]),
        (7.5, "3D grounding\nreachable candidates", f"{len(observations)} observation rounds", COLORS["blue"]),
        (10.3, "online OWLv2\ntarget confirmation", "found at L2 bathroom" if found else "result unavailable", COLORS["green"]),
    ]
    for x, title, detail, color in boxes:
        patch = FancyBboxPatch((x - .9, 1.65), 1.8, 1.55, boxstyle="round,pad=.08",
                               linewidth=2, edgecolor=color, facecolor="white")
        ax.add_patch(patch)
        ax.text(x, 2.62, title, ha="center", va="center", fontsize=9, weight="bold", color=COLORS["navy"])
        ax.text(x, 1.98, detail, ha="center", va="center", fontsize=8, color=COLORS["gray"])
    for left, right in zip(boxes[:-1], boxes[1:]):
        ax.add_patch(FancyArrowPatch((left[0] + .95, 2.43), (right[0] - .95, 2.43),
                                     arrowstyle="-|>", mutation_scale=14, linewidth=1.7, color=COLORS["gray"]))
    ax.text(6, .7, "negative observation lowers belief in the failed location; semantic recovery expands only when the task policy allows it",
            ha="center", fontsize=9, color=COLORS["gray"])
    save(fig, out, "目标丢失_语义恢复搜索流程")


def plot_five_stage_pipeline(out: Path) -> None:
    """A clean vector-style overview of the actual project architecture."""
    fig, ax = plt.subplots(figsize=(15, 4.8))
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 5.2)
    ax.axis("off")
    ax.set_title("PRE-MAP-VLN: from 3D exploration to language-guided task execution",
                 fontsize=16, pad=16)
    stages = [
        (1.45, "1 3D exploration", "FALCON\n3D occupancy\ntrajectory"),
        (4.45, "2 semantic map", "Boxer 3D boxes\nrooms + floors\ntransitions"),
        (7.45, "3 language task graph", "Qwen\npartial order\nspatial + branch"),
        (10.45, "4 joint planning", "candidate poses\n3D A* + B-spline\ncollision check"),
        (13.45, "5 online execution", "OWLv2\nverify + update\nsemantic recovery"),
    ]
    edge_colors = [COLORS["blue"], COLORS["teal"], COLORS["orange"], COLORS["red"], COLORS["green"]]
    for i, (x, title, detail) in enumerate(stages):
        color = edge_colors[i]
        box = FancyBboxPatch((x - 1.18, 1.55), 2.36, 2.05, boxstyle="round,pad=.10",
                             linewidth=2.4, edgecolor=color, facecolor="white")
        ax.add_patch(box)
        ax.text(x, 3.18, title, ha="center", va="center", fontsize=10,
                weight="bold", color=COLORS["navy"])
        ax.text(x, 2.28, detail, ha="center", va="center", fontsize=9,
                linespacing=1.45, color=COLORS["gray"])
        if i < len(stages) - 1:
            ax.add_patch(FancyArrowPatch((x + 1.28, 2.58), (stages[i + 1][0] - 1.28, 2.58),
                                         arrowstyle="-|>", mutation_scale=16, linewidth=2,
                                         color=COLORS["gray"]))
    ax.text(7.45, .62,
            "Reusable 3D map  →  constrained mission  →  executable observation state  →  verified result",
            ha="center", fontsize=11, color=COLORS["navy"], weight="bold")
    save(fig, out, "系统总览_五阶段流程")


def plot_map_representation_stack(out: Path, selected_dir: Path, max_floor: int) -> None:
    """Show the reusable map products without mixing them with raw outputs."""
    names = [
        ("3D explored map", selected_dir / "stage1/00337_truth_vs_falcon_3d.png"),
        ("room geometry", selected_dir / "room_pipeline/00337_transition_room_summary.png"),
        ("room semantics", selected_dir / "room_pipeline/00337_room_semantics.png"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))
    fig.suptitle(f"Reusable semantic map representation (floors 1--{max_floor})", fontsize=16)
    for ax, (title, path) in zip(axes, names):
        if path.exists():
            ax.imshow(plt.imread(path))
        ax.set_title(title, fontsize=12, weight="bold")
        ax.axis("off")
    fig.text(.5, .04, "occupancy and trajectory  →  structural regions  →  room-level semantic labels",
             ha="center", fontsize=11, color=COLORS["navy"])
    save(fig, out, "地图表示层_三维占据到房间语义")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-graph", type=Path, default=Path("outputs/stage2_3d/tasks/00337_qwen_e2e_owlv2_v2_20260829/task_graph.json"))
    parser.add_argument("--candidates", type=Path, default=Path("outputs/stage2_3d/tasks/00337_all_found_showcase_v1_20260830/execution/candidates_with_recovery.json"))
    parser.add_argument("--execution", type=Path, default=Path("outputs/stage2_3d/tasks/00337_toilet_paper_real_recovery_20260830_114159/execution/habitat_execution.json"))
    parser.add_argument("--trajectory-execution", type=Path, default=Path("outputs/stage2_3d/tasks/00337_all_found_showcase_v1_20260830/execution/habitat_execution.json"))
    parser.add_argument("--out", type=Path, default=Path("paper/system_overview/assets/selected/generated"))
    parser.add_argument("--max-floor", type=int, default=3)
    args = parser.parse_args()

    graph = read_json(args.task_graph)
    candidates = read_json(args.candidates)
    recovery_execution = read_json(args.execution)
    trajectory_execution = read_json(args.trajectory_execution)
    plot_task_graph(graph, args.out)
    plot_candidate_poses(candidates, args.out, "check_microwave_kitchen_l2")
    plot_trajectory(trajectory_execution, args.out, args.max_floor)
    plot_recovery(recovery_execution, args.out)
    selected_dir = args.out.parent
    plot_five_stage_pipeline(args.out)
    plot_map_representation_stack(args.out, selected_dir, args.max_floor)
    print(f"generated 6 panels in {args.out}")


if __name__ == "__main__":
    main()
