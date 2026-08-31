#!/usr/bin/env python3
"""Render a data-native cross-floor UAV path section for HM3D scene 00337."""
from __future__ import annotations

import argparse, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", type=Path, default=Path("PRE_MAP_VLN/outputs/stage2_3d/00337_scene_graph.json"))
    ap.add_argument("--execution", type=Path, default=Path("PRE_MAP_VLN/outputs/stage2_3d/validation/00337_multifloor_habitat_clearance010_bspline_final_v4/habitat_execution.json"))
    ap.add_argument("--floors", type=Path, default=Path("PRE_MAP_VLN/outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix/floors.json"))
    ap.add_argument("--output", type=Path, default=Path("PRE_MAP_VLN/outputs/stage2_3d/validation/00337_multifloor_habitat_clearance010_bspline_final_v4/00337_cross_floor_section.png"))
    args = ap.parse_args()
    scene, trace, floors = json.loads(args.scene.read_text()), json.loads(args.execution.read_text()), json.loads(args.floors.read_text())
    traj = np.asarray(trace["trajectory_xyz_yaw"], float)
    # cumulative horizontal distance is the natural abscissa for a path longitudinal section
    dxy = np.linalg.norm(np.diff(traj[:, :2], axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(dxy)]
    floor_by_id = {int(x["floor"]): x for x in floors}
    colors = {1: "#3b82f6", 2: "#22c55e", 3: "#a855f7"}

    fig = plt.figure(figsize=(16, 10), constrained_layout=True)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.18, 1], height_ratios=[1, 1.05])
    ax3 = fig.add_subplot(gs[:, 0], projection="3d")
    axp = fig.add_subplot(gs[0, 1])
    axs = fig.add_subplot(gs[1, 1])

    # 3-D context: room footprints at their floor elevations, stair centerlines, and UAV trajectory
    for room in scene["rooms"]:
        fid = int(room["floor_id"]); poly = np.asarray(room["polygon_xy_m"], float)
        z = floor_by_id[fid]["floor_z_m"]
        ax3.plot(poly[:, 0], poly[:, 1], z, color=colors[fid], alpha=.18, linewidth=.45)
    for tr in scene.get("transitions", []):
        c = np.asarray(tr["centerline_xyz"], float)
        ax3.plot(c[:, 0], c[:, 1], c[:, 2], color="#f59e0b", linewidth=2.5, alpha=.9)
    ax3.plot(traj[:, 0], traj[:, 1], traj[:, 2], color="#111827", linewidth=2.2, label="UAV trajectory")
    ax3.scatter(*traj[0, :3], color="#16a34a", s=55, label="start", depthshade=False)
    ax3.scatter(*traj[-1, :3], color="#dc2626", s=55, label="finish", depthshade=False)
    ax3.set_title("Scene 00337: 3-D geometry and UAV cross-floor path")
    ax3.set_xlabel("x [m]"); ax3.set_ylabel("y [m]"); ax3.set_zlabel("z [m]")
    ax3.view_init(elev=25, azim=-62); ax3.legend(loc="upper left", fontsize=8)

    # plan locator, colored by floor and with the actual path projection
    for room in scene["rooms"]:
        fid = int(room["floor_id"]); poly = np.asarray(room["polygon_xy_m"], float)
        axp.add_patch(Polygon(poly, closed=True, facecolor=colors[fid], edgecolor=colors[fid], alpha=.10, linewidth=.4))
    axp.plot(traj[:, 0], traj[:, 1], color="#111827", linewidth=1.8)
    axp.scatter(traj[0, 0], traj[0, 1], c="#16a34a", s=42, zorder=4)
    axp.scatter(traj[-1, 0], traj[-1, 1], c="#dc2626", s=42, zorder=4)
    for tr in scene.get("transitions", []):
        c = np.asarray(tr["centerline_xyz"], float); axp.plot(c[:, 0], c[:, 1], "--", color="#f59e0b", linewidth=1.5)
    axp.set_aspect("equal", adjustable="box"); axp.set_title("Plan locator (color=floor, dashed=stair centerline)")
    axp.set_xlabel("x [m]"); axp.set_ylabel("y [m]"); axp.grid(alpha=.2)

    # longitudinal section with floor slabs, flight bands and floor transitions
    for fid, f in floor_by_id.items():
        z = float(f["floor_z_m"]); hz = float(f["flight_z_m"])
        axs.axhspan(z - .10, z + .10, color=colors[fid], alpha=.18)
        axs.axhline(z, color=colors[fid], linewidth=1.2)
        axs.axhline(hz, color=colors[fid], linestyle=":", linewidth=.9, alpha=.8)
        axs.text(s[-1] * 1.01, z, f"L{fid} floor {z:.2f} m", color=colors[fid], va="center", fontsize=8)
        axs.text(s[-1] * 1.01, hz, f"flight {hz:.2f} m", color=colors[fid], va="center", fontsize=7)
    axs.plot(s, traj[:, 2], color="#111827", linewidth=2.4, label="UAV z(s)")
    # segment boundaries from authoritative validation trace
    for seg in trace.get("segments", []):
        i0, i1 = int(seg["start_frame"]), int(seg["end_frame"])
        axs.axvline(s[i0], color="#64748b", linewidth=.8, alpha=.7)
        axs.text(s[min(i0 + max(2, (i1-i0)//2), len(s)-1)], max(traj[:,2]) + .28, seg["task_id"], fontsize=7, ha="center", rotation=12)
    axs.scatter(s[0], traj[0,2], c="#16a34a", s=42, zorder=5); axs.scatter(s[-1], traj[-1,2], c="#dc2626", s=42, zorder=5)
    axs.set_title(f"Longitudinal path section: horizontal distance {s[-1]:.1f} m, z {traj[:,2].min():.2f}–{traj[:,2].max():.2f} m")
    axs.set_xlabel("Horizontal cumulative distance s [m]"); axs.set_ylabel("Altitude z [m]"); axs.grid(alpha=.2); axs.set_xlim(0, s[-1]*1.12); axs.legend(fontsize=8)
    fig.suptitle("HM3D 00337 | UAV cross-floor path section (data-native rendering)", fontsize=16, fontweight="bold")
    args.output.parent.mkdir(parents=True, exist_ok=True); fig.savefig(args.output, dpi=220, facecolor="white"); print(args.output)


if __name__ == "__main__": main()
