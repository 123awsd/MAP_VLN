#!/usr/bin/env python3
"""Render a compact trajectory, task-state, and terminal-evidence report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


COLORS = {1: "#3b82f6", 2: "#22c55e", 3: "#a855f7", 4: "#f97316"}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("execution_dir", type=Path)
    parser.add_argument("scene_graph", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    trace = json.loads((args.execution_dir / "habitat_execution.json").read_text())
    scene = json.loads(args.scene_graph.read_text())
    output = args.output or args.execution_dir / "task_execution_summary.png"
    observations = trace.get("observations", [])
    figure = plt.figure(figsize=(18, 10), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=(1.25, 1.0), height_ratios=(1.0, 1.0))

    axis_3d = figure.add_subplot(grid[:, 0], projection="3d")
    trajectory = np.asarray(trace.get("trajectory_xyz_yaw", []), dtype=float)
    if trajectory.size:
        axis_3d.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color="#0f172a", linewidth=2.5, zorder=5)
        axis_3d.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], color="#22c55e", s=70, label="start")
        axis_3d.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], color="#ef4444", s=70, label="finish")
    visited_ids = {item.get("candidate_id"): item for item in observations}
    selected_object_ids = {
        visit.get("object_id")
        for replan in trace.get("replans", [])
        for visit in replan.get("plan", {}).get("visits", [])[:1]
    }
    for room in scene.get("rooms", []):
        floor = int(room.get("floor_id", 0) or 0)
        for obj in room.get("objects", []):
            if obj.get("id") not in selected_object_ids:
                continue
            center = obj["center_xyz_m"]
            axis_3d.scatter(*center, color=COLORS.get(floor, "#eab308"), marker="s", s=85)
            axis_3d.text(*center, obj.get("label", "object"), fontsize=8)
    axis_3d.set_title("3-D execution trajectory and inspected objects")
    axis_3d.set_xlabel("Falcon x [m]")
    axis_3d.set_ylabel("Falcon y [m]")
    axis_3d.set_zlabel("Falcon z [m]")
    axis_3d.legend(loc="upper left")
    axis_3d.view_init(elev=24, azim=-62)

    axis_state = figure.add_subplot(grid[0, 1])
    labels = [item["task_id"].replace("inspect_", "").replace("find_", "") for item in observations]
    colors = ["#22c55e" if item.get("verification_outcome") == "found" else "#ef4444" for item in observations]
    axis_state.barh(np.arange(len(labels)), np.ones(len(labels)), color=colors, alpha=0.85)
    axis_state.set_yticks(np.arange(len(labels)), labels)
    axis_state.invert_yaxis()
    axis_state.set_xlim(0, 1.2)
    axis_state.set_xticks([])
    for index, item in enumerate(observations):
        axis_state.text(0.03, index, item.get("outcome", "unknown"), va="center", color="white", fontweight="bold")
    audit = trace.get("clearance_audit") or {}
    axis_state.set_title(
        f"Task state: {trace.get('status')} | path {trace.get('path_length_m', 0):.1f} m"
        + (f" | min ESDF {audit.get('minimum_esdf_m', 0):.2f} m" if audit else "")
    )

    axis_evidence = figure.add_subplot(grid[1, 1])
    axis_evidence.axis("off")
    thumbnails = []
    captions = []
    for item in observations[-6:]:
        path = args.execution_dir / item.get("rgb", "")
        if path.is_file():
            image = Image.open(path).convert("RGB")
            image.thumbnail((260, 180))
            thumbnails.append(np.asarray(image))
            captions.append(f"{item['task_id']}\n{item.get('verification_outcome', item.get('outcome'))}")
    if thumbnails:
        cells = []
        width = max(image.shape[1] for image in thumbnails)
        height = max(image.shape[0] for image in thumbnails)
        for image in thumbnails:
            canvas = np.full((height, width, 3), 245, dtype=np.uint8)
            canvas[:image.shape[0], :image.shape[1]] = image
            cells.append(canvas)
        while len(cells) < 6:
            cells.append(np.full((height, width, 3), 245, dtype=np.uint8))
        montage = np.concatenate((np.concatenate(cells[:3], axis=1), np.concatenate(cells[3:6], axis=1)), axis=0)
        axis_evidence.imshow(montage)
        for index, caption in enumerate(captions):
            row, column = divmod(index, 3)
            axis_evidence.text((column + 0.5) / 3, 0.50 - row * 0.50, caption, transform=axis_evidence.transAxes, ha="center", va="top", fontsize=7, bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"})
    axis_evidence.set_title("Terminal RGB evidence (latest tasks)")
    figure.suptitle("PRE_MAP_VLN Stage-2 multi-floor task execution", fontsize=16)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, facecolor="#ffffff")
    print(f"saved={output}")


if __name__ == "__main__":
    main()
