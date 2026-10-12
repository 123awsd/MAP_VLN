#!/usr/bin/env python3
"""Plot an offline real-map FALCON route and approved room boundaries."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--scene-graph", type=Path, required=True)
    p.add_argument("--boundary", type=Path)
    p.add_argument("--trajectory", type=Path, required=True)
    p.add_argument("--result", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    scene = json.loads(a.scene_graph.read_text())
    result = json.loads(a.result.read_text())
    traj = np.loadtxt(a.trajectory, delimiter=",", skiprows=1)
    if traj.ndim == 1:
        traj = traj[None, :]
    fig, ax = plt.subplots(figsize=(13, 9), constrained_layout=True)
    excluded = set(result["excluded_rooms"])
    for room in scene["rooms"]:
        poly = np.asarray(room["polygon_xy_m"] + [room["polygon_xy_m"][0]])
        off = room["name"] in excluded
        ax.fill(poly[:, 0], poly[:, 1], color="#e5e7eb" if not off else "#fecaca", alpha=.45)
        ax.plot(poly[:, 0], poly[:, 1], color="#6b7280" if not off else "#dc2626", lw=1.2)
        c = room["centroid_xy_m"]
        ax.text(c[0], c[1], room["name"], ha="center", va="center", fontsize=8)
    if a.boundary:
        boundary = json.loads(a.boundary.read_text())
        poly = np.asarray(boundary["allowed_polygon_xy_m"])
        poly = np.vstack((poly, poly[0]))
        ax.plot(poly[:, 0], poly[:, 1], color="#7c3aed", lw=2.2, label="approved boundary")
        ax.fill(poly[:, 0], poly[:, 1], color="#a78bfa", alpha=0.08)
        for index, item in enumerate(boundary.get("excluded_polygons", []), start=1):
            vertices = item.get("polygon_xy_m", item) if isinstance(item, dict) else item
            blocked = np.asarray(vertices)
            blocked = np.vstack((blocked, blocked[0]))
            ax.fill(blocked[:, 0], blocked[:, 1], color="#ef4444", alpha=0.25)
            ax.plot(
                blocked[:, 0], blocked[:, 1], color="#dc2626", lw=2.0,
                label="no-fly zone" if index == 1 else None,
            )
    if len(traj):
        label = "hybrid offline coverage route" if result.get("coverage_fallback") else "FALCON simulated route"
        ax.plot(traj[:, 1], traj[:, 2], color="#075985", lw=2.0, label=label)
        ax.scatter(traj[0, 1], traj[0, 2], s=70, c="#16a34a", marker="o", label="start", zorder=5)
        ax.scatter(traj[-1, 1], traj[-1, 2], s=90, c="#f59e0b", marker="*", label="end", zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=.18)
    ax.set_xlabel("world X (m)")
    ax.set_ylabel("world Y (m)")
    if result.get("boundary_only"):
        title = "Offline exploration preview — boundary constrained"
    else:
        title = "Offline FALCON exploration — " + (
            "all requested rooms visited" if result["all_target_rooms_visited"] else "incomplete"
        )
    ax.set_title(title)
    ax.legend(loc="best")
    fig.savefig(a.output, dpi=180)
    print(a.output)


if __name__ == "__main__":
    main()
