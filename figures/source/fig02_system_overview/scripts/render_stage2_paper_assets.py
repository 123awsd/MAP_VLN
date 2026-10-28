#!/usr/bin/env python3
"""Render data-faithful Stage-2 paper assets without flowchart-style graphics."""

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
from matplotlib.patches import FancyArrowPatch, Rectangle
from mpl_toolkits.mplot3d.art3d import Line3DCollection


ROOT = Path(__file__).resolve().parents[3]
STAGE1 = ROOT / "outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829"
PCD = STAGE1 / "bag_export/map_occupied.pcd"
FLOORS_JSON = ROOT / "outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix/floors.json"
TASK_GRAPH_RUN = ROOT / "outputs/stage2_3d/tasks/00337_qwen_e2e_owlv2_v2_20260829"
JOINT_RUN = ROOT / "outputs/stage2_3d/tasks/00337_all_found_showcase_v1_20260830"
RECOVERY_RUN = ROOT / "outputs/stage2_3d/tasks/00337_toilet_paper_real_recovery_v4_20260831"
CANDIDATE_OUT = ROOT / "paper/system_overview/assets/candidates/stage2_real_data"
SELECTED_OUT = ROOT / "paper/system_overview/assets/selected/stage2_real_data"

FLOOR_COLORS = {1: "#337CB7", 2: "#2AA17E", 3: "#E18447"}
TASK_COLORS = {
    "check_microwave_kitchen_l2": "#D9822B",
    "check_coffee_table_living_l2": "#7B61A8",
    "check_tv_living_l3": "#2878B5",
    "check_toilet_paper_bath_l2": "#2A9D8F",
}
TASK_LABELS = {
    "check_microwave_kitchen_l2": "Microwave · L2",
    "check_coffee_table_living_l2": "Coffee table · L2",
    "check_tv_living_l3": "Television · L3",
    "check_toilet_paper_bath_l2": "Toilet paper · L2",
}

CJK_REGULAR = font_manager.FontProperties(
    fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"
)
CJK_BOLD = font_manager.FontProperties(
    fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
)


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_ascii_pcd(path: Path) -> np.ndarray:
    header = 0
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            header += 1
            if line.strip().lower().startswith("data"):
                break
    return np.loadtxt(path, dtype=np.float32, skiprows=header, usecols=(0, 1, 2))


def sample(points: np.ndarray, maximum: int, seed: int) -> np.ndarray:
    if len(points) <= maximum:
        return points
    rng = np.random.default_rng(seed)
    return points[rng.choice(len(points), maximum, replace=False)]


def floor_data() -> tuple[list[dict], np.ndarray]:
    floors = load_json(FLOORS_JSON)[:3]
    z = np.asarray([item["floor_z_m"] for item in floors])
    bounds = np.asarray([z[0] - 0.35, (z[0] + z[1]) / 2,
                         (z[1] + z[2]) / 2, z[2] + 2.85])
    return floors, bounds


def save(fig, stem: str, *, tight: bool = True):
    CANDIDATE_OUT.mkdir(parents=True, exist_ok=True)
    save_kwargs = {
        "dpi": 360,
        "facecolor": "white",
        "transparent": False,
    }
    if tight:
        save_kwargs.update({"bbox_inches": "tight", "pad_inches": 0.035})
    fig.savefig(CANDIDATE_OUT / f"{stem}.png", **save_kwargs)
    fig.savefig(CANDIDATE_OUT / f"{stem}.pdf", **save_kwargs)
    plt.close(fig)


def clean_axis(ax):
    ax.set_axis_off()
    ax.set_facecolor("white")


def draw_arrow(ax, start, end, color="#748093", dashed=False, curve=0.0, width=1.2):
    patch = FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=11, linewidth=width,
        color=color, linestyle="--" if dashed else "-",
        connectionstyle=f"arc3,rad={curve}", shrinkA=19, shrinkB=19,
        transform=ax.transData, zorder=1,
    )
    ax.add_patch(patch)


def render_language_task_graph():
    data = load_json(TASK_GRAPH_RUN / "task_graph.json")
    tasks = {item["id"]: item for item in data["tasks"]}
    positions = {
        "inspect_sink_l1_bathroom": (0.10, 0.53),
        "confirm_microwave_l2_kitchen": (0.32, 0.73),
        "inspect_coffee_table_l2_living_room": (0.32, 0.51),
        "inspect_lamp_l3_bedroom": (0.32, 0.29),
        "confirm_tv_l3_living_room": (0.61, 0.51),
        "find_tv_l2_living_room_backup": (0.86, 0.51),
    }
    short = {
        "inspect_sink_l1_bathroom": ("SINK", "L1 · bathroom", "inspect"),
        "confirm_microwave_l2_kitchen": ("MICROWAVE", "L2 · kitchen", "confirm"),
        "inspect_coffee_table_l2_living_room": ("COFFEE TABLE", "L2 · near sofa", "inspect"),
        "inspect_lamp_l3_bedroom": ("LAMP", "L3 · near bed", "inspect"),
        "confirm_tv_l3_living_room": ("TELEVISION", "L3 · near fireplace", "confirm"),
        "find_tv_l2_living_room_backup": ("TELEVISION", "L2 · backup search", "find"),
    }

    fig, ax = plt.subplots(figsize=(13.2, 6.2), facecolor="white")
    clean_axis(ax)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    instruction = data["instruction"]
    ax.text(0.055, 0.92, "“" + instruction + "”", fontsize=13.2,
            fontproperties=CJK_REGULAR, color="#263442", va="top")
    ax.text(0.055, 0.835, "Qwen3.7-plus  ·  structured task graph",
            fontsize=9.3, color="#7D8794", va="top")

    for task_id, task in tasks.items():
        for prerequisite in task.get("prerequisites", []):
            curve = 0.17 if (prerequisite == "inspect_sink_l1_bathroom" and
                             task_id == "confirm_tv_l3_living_room") else 0.0
            draw_arrow(ax, positions[prerequisite], positions[task_id],
                       color="#8D99A8", curve=curve)
    for rule in data.get("conditional_rules", []):
        for activated in rule.get("activate_task_ids", []):
            draw_arrow(ax, positions[rule["source_task_id"]], positions[activated],
                       color="#C9585D", dashed=True, curve=-0.10, width=1.35)
            midpoint = 0.5 * (np.asarray(positions[rule["source_task_id"]]) +
                              np.asarray(positions[activated]))
            ax.text(midpoint[0], midpoint[1] + 0.055, "if NOT FOUND", color="#C9585D",
                    fontsize=8.0, ha="center", fontweight="bold")

    for task_id, (x, y) in positions.items():
        task = tasks[task_id]
        floor = int(task["target"]["floor_id"])
        inactive = not task.get("active_initially", True)
        color = FLOOR_COLORS[floor]
        ax.scatter([x], [y], s=1580, facecolor="white" if inactive else color,
                   edgecolor=color, linewidth=2.0, zorder=3,
                   linestyle="--" if inactive else "-")
        label, context, action = short[task_id]
        compact_label = label.replace("MICROWAVE", "MICRO\nWAVE").replace(
            "COFFEE TABLE", "COFFEE\nTABLE").replace("TELEVISION", "TELE\nVISION")
        ax.text(x, y + 0.009, compact_label, ha="center", va="center", fontsize=7.7,
                color=color if inactive else "white", fontweight="bold", zorder=4)
        ax.text(x, y - 0.100, context, ha="center", va="top", fontsize=8.8,
                color="#344455")
        relation = task.get("spatial_constraints", {}).get("relation")
        detail = action.upper() + (f" · {relation}" if relation else "")
        ax.text(x, y - 0.133, detail, ha="center", va="top", fontsize=7.3,
                color="#8B95A2", fontweight="bold")

    ax.text(0.055, 0.095, "6 semantic tasks", fontsize=8.6, color="#657281")
    ax.text(0.215, 0.095, "5 precedence constraints", fontsize=8.6, color="#657281")
    ax.text(0.425, 0.095, "1 outcome-conditioned branch", fontsize=8.6, color="#C9585D")
    ax.plot([0.055, 0.92], [0.135, 0.135], color="#DDE2E7", linewidth=0.8)
    save(fig, "language_to_semantic_task_graph")


def pose_dict(value) -> dict:
    if isinstance(value, dict):
        return value
    return {"x": value[0], "y": value[1], "z": value[2], "yaw": value[3]}


def frustum_segments(pose, hfov_deg: float, vfov_deg: float, depth: float) -> list[np.ndarray]:
    pose = pose_dict(pose)
    origin = np.asarray([pose["x"], pose["y"], pose["z"]], dtype=float)
    yaw = float(pose["yaw"])
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0])
    vertical = np.asarray([0.0, 0.0, 1.0])
    half_w = depth * math.tan(math.radians(hfov_deg) * 0.5)
    half_h = depth * math.tan(math.radians(vfov_deg) * 0.5)
    center = origin + forward * depth
    corners = [center + sx * right * half_w + sz * vertical * half_h
               for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    segments = [np.vstack([origin, corner]) for corner in corners]
    segments += [np.vstack([corners[i], corners[(i + 1) % 4]]) for i in range(4)]
    return segments


def fit_3d(ax, points: np.ndarray, z_scale=0.88, pad=0.03):
    lo = points.min(axis=0); hi = points.max(axis=0)
    span = np.maximum(hi - lo, 1e-3)
    ax.set_xlim(lo[0] - span[0] * pad, hi[0] + span[0] * pad)
    ax.set_ylim(lo[1] - span[1] * pad, hi[1] + span[1] * pad)
    ax.set_zlim(lo[2] - span[2] * pad, hi[2] + span[2] * pad)
    ax.set_box_aspect((span[0], span[1], span[2] * z_scale))


def open_map_points(points: np.ndarray) -> tuple[np.ndarray, dict[int, np.ndarray]]:
    floors, bounds = floor_data()
    selected = []
    by_floor = {}
    x_front = np.quantile(points[:, 0], 0.84)
    y_front = np.quantile(points[:, 1], 0.14)
    for index, info in enumerate(floors):
        mask = ((points[:, 2] >= bounds[index]) & (points[:, 2] < bounds[index + 1]) &
                (points[:, 2] <= float(info["floor_z_m"]) + 2.35) &
                ~((points[:, 0] > x_front) | (points[:, 1] < y_front)))
        floor_points = sample(points[mask], 36000, 100 + index)
        by_floor[index + 1] = floor_points
        selected.append(floor_points)
    return np.vstack(selected), by_floor


def render_joint_optimization():
    candidates = load_json(JOINT_RUN / "execution/candidates_with_recovery.json")["by_task"]
    execution = load_json(JOINT_RUN / "execution/habitat_execution.json")
    points = load_ascii_pcd(PCD)
    cloud, by_floor = open_map_points(points)

    fig = plt.figure(figsize=(11.6, 6.8), facecolor="white")
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    clean_axis(ax); ax.set_proj_type("ortho")
    for floor, floor_points in by_floor.items():
        ax.scatter(floor_points[:, 0], floor_points[:, 1], floor_points[:, 2],
                   s=0.38, c=FLOOR_COLORS[floor], alpha=0.19,
                   linewidths=0, depthshade=False, rasterized=True)

    candidate_lookup = {}
    feasible_count = 0
    for task_id, task_candidates in candidates.items():
        color = TASK_COLORS[task_id]
        for candidate in task_candidates:
            candidate_lookup[candidate["id"]] = candidate
            feasible_count += 1
            ax.add_collection3d(Line3DCollection(
                frustum_segments(candidate["pose"], candidate.get("horizontal_fov_deg", 90),
                                 candidate.get("vertical_fov_deg", 70), 0.42),
                colors=color, linewidths=0.38, alpha=0.18,
            ))

    route_points = []
    for sequence, segment in enumerate(execution["executed_segments"], 1):
        task_id = segment["task_id"]
        path = np.asarray(segment["bspline_points_xyz_m"], dtype=float)
        route_points.append(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color="white", linewidth=4.4,
                alpha=0.94, solid_capstyle="round", zorder=8)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color=TASK_COLORS[task_id],
                linewidth=2.25, alpha=0.98, solid_capstyle="round", zorder=9)

    for sequence, observation in enumerate(execution["observations"], 1):
        task_id = observation["task_id"]
        color = TASK_COLORS[task_id]
        candidate = candidate_lookup.get(observation["candidate_id"], {})
        pose = observation["pose"]
        ax.add_collection3d(Line3DCollection(
            frustum_segments(pose, candidate.get("horizontal_fov_deg", 90),
                             candidate.get("vertical_fov_deg", 70), 0.68),
            colors=color, linewidths=1.45, alpha=0.98,
        ))
        ax.scatter([pose[0]], [pose[1]], [pose[2]], s=34, color=color,
                   edgecolor="white", linewidth=0.8, depthshade=False, zorder=12)
        ax.text(pose[0], pose[1], pose[2] + 0.30, str(sequence), color=color,
                fontsize=9, fontweight="bold", ha="center", zorder=13)

    all_extent = np.vstack([cloud] + route_points)
    fit_3d(ax, all_extent, z_scale=0.88)
    ax.view_init(elev=25, azim=-58)
    ax.set_position([0.00, 0.00, 1.00, 1.00])
    ax.dist = 6.7
    handles = [Line2D([0], [0], color=TASK_COLORS[key], lw=2.8, label=TASK_LABELS[key])
               for key in TASK_COLORS]
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.02, 0.98),
              frameon=False, fontsize=8.5, handlelength=2.4)
    ax.text2D(0.03, 0.045,
              f"{len(TASK_COLORS)} tasks   ·   {feasible_count} feasible views   ·   "
              f"{execution['path_length_m']:.1f} m executed   ·   3D A* + B-spline",
              transform=ax.transAxes, fontsize=8.6, color="#596878")
    save(fig, "joint_task_viewpoint_path_optimization", tight=False)


def contiguous_segments(points: np.ndarray, mask: np.ndarray) -> list[np.ndarray]:
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    breaks = np.flatnonzero(np.diff(indices) > 1) + 1
    return [points[group] for group in np.split(indices, breaks) if len(group) > 1]


def draw_floor_map(ax, points, trajectory, observations, candidates, floor, title):
    floors, _ = floor_data()
    z0 = float(floors[floor - 1]["floor_z_m"])
    cloud_mask = (points[:, 2] >= z0 + 0.20) & (points[:, 2] <= z0 + 2.20)
    cloud = sample(points[cloud_mask], 70000, 200 + floor)
    ax.scatter(cloud[:, 0], cloud[:, 1], s=0.34, color="#8797A7", alpha=0.32,
               linewidths=0, rasterized=True)
    traj_mask = np.abs(trajectory[:, 2] - (1.05 if floor == 1 else 3.95)) < 1.35
    for segment in contiguous_segments(trajectory, traj_mask):
        ax.plot(segment[:, 0], segment[:, 1], color="#34495E", linewidth=1.45,
                alpha=0.76, solid_capstyle="round")

    for candidate in candidates:
        if int(candidate.get("floor_id", -1)) != floor:
            continue
        pose = candidate["pose"]
        location = candidate.get("location_hypothesis_id", "")
        if "L1_boxer_13" in location:
            color = "#C9585D"
        elif "L1_boxer_3" in location:
            color = "#D9972F"
        else:
            color = "#2A9D78"
        ax.scatter([pose["x"]], [pose["y"]], s=18, facecolor="white",
                   edgecolor=color, linewidth=0.8, alpha=0.75)

    for observation in observations:
        pose = observation["pose"]
        obs_floor = 1 if pose[2] < 2.0 else 2
        if obs_floor != floor:
            continue
        outcome = observation["outcome"]
        color = "#2A9D78" if outcome == "found" else (
            "#D9972F" if "region_L1_R3" in observation["candidate_id"] else "#C9585D")
        ax.scatter([pose[0]], [pose[1]], s=48, color=color, edgecolor="white",
                   linewidth=0.9, zorder=5)
        ax.text(pose[0], pose[1] + 0.22, str(observation["sequence"] + 1),
                color=color, fontsize=7.5, ha="center", fontweight="bold")

    ax.set_aspect("equal", adjustable="box")
    clean_axis(ax)
    ax.set_title(title, loc="left", fontsize=10.2, color="#334455", pad=5,
                 fontweight="bold")


def render_active_recovery():
    execution = load_json(RECOVERY_RUN / "execution/habitat_execution.json")
    candidates = load_json(RECOVERY_RUN / "execution/candidates_with_recovery.json")["by_task"]
    candidates = next(iter(candidates.values()))
    points = load_ascii_pcd(PCD)
    trajectory = np.asarray(execution["trajectory_xyz_yaw"], dtype=float)
    observations = execution["observations"]
    selected_sequences = [0, 2, 3, 6]
    frame_titles = (
        "Expected site · first view",
        "Expected site · 3 views exhausted",
        "Semantic alternative · not found",
        "Historical target · recovered",
    )
    border_colors = ("#C9585D", "#C9585D", "#D9972F", "#2A9D78")

    fig = plt.figure(figsize=(14.8, 8.4), facecolor="white")
    outer = fig.add_gridspec(2, 1, height_ratios=[0.94, 1.12], hspace=0.25)
    top = outer[0].subgridspec(1, 4, wspace=0.035)
    for index, sequence in enumerate(selected_sequences):
        observation = observations[sequence]
        image_path = RECOVERY_RUN / "execution" / observation["rgb"]
        image = plt.imread(image_path)
        ax = fig.add_subplot(top[0, index])
        ax.imshow(image)
        ax.set_xticks([]); ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_linewidth(2.0); spine.set_color(border_colors[index])
        ax.set_title(frame_titles[index], fontsize=9.3, color="#334455", pad=6,
                     fontweight="bold")
        if observation["outcome"] == "found":
            detections = observation["verification"]["owlv2"]["detections"]
            if detections:
                detection = detections[0]
                x1, y1, x2, y2 = detection["bbox_xyxy"]
                ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1,
                                       fill=False, edgecolor="#18A875", linewidth=2.4))
                ax.text(x1, max(12, y1 - 8),
                        f"toilet paper  {detection['score']:.2f}", color="white",
                        fontsize=8.0, fontweight="bold",
                        bbox={"facecolor": "#18A875", "edgecolor": "none", "pad": 2.0})
        else:
            ax.text(0.035, 0.94, "NO DETECTION", transform=ax.transAxes, va="top",
                    color="white", fontsize=7.6, fontweight="bold",
                    bbox={"facecolor": border_colors[index], "edgecolor": "none", "pad": 2.2})

    bottom = outer[1].subgridspec(1, 3, width_ratios=[1.0, 0.56, 1.0], wspace=0.12)
    ax_l1 = fig.add_subplot(bottom[0, 0])
    draw_floor_map(ax_l1, points, trajectory, observations, candidates, 1,
                   "L1 · expected site + semantic alternative")
    ax_l2 = fig.add_subplot(bottom[0, 2])
    draw_floor_map(ax_l2, points, trajectory, observations, candidates, 2,
                   "L2 · recovered target")

    ax_score = fig.add_subplot(bottom[0, 1])
    clean_axis(ax_score)
    event = execution["semantic_recovery"]["events"][0]
    hypotheses = event["plan"]["hypotheses"]
    labels = ["historical target", "toilet anchor", "bathtub support"]
    colors = ["#2A9D78", "#4A8DB7", "#D9972F"]
    scores = [float(item["joint_recovery_score"]) for item in hypotheses]
    order = np.arange(len(scores))[::-1]
    for y, label, score, color in zip(order, labels, scores, colors):
        ax_score.plot([0, score], [y, y], color=color, linewidth=1.5, alpha=0.75)
        ax_score.scatter([score], [y], s=44, color=color, edgecolor="white", linewidth=0.7)
        ax_score.text(0, y + 0.20, label, fontsize=8.0, color="#4E5D6C", ha="left")
        ax_score.text(score + 0.035, y, f"{score:.2f}", fontsize=8.2, color=color,
                      va="center", fontweight="bold")
    ax_score.set_xlim(-0.03, max(scores) + 0.28)
    ax_score.set_ylim(-0.55, len(scores) - 0.15)
    ax_score.text(0, len(scores) - 0.22, "Qwen recovery hypotheses",
                  fontsize=9.4, color="#334455", fontweight="bold")
    ax_score.text(0, -0.47, "location-level budget\n3 viewpoints / location",
                  fontsize=7.7, color="#7C8794", linespacing=1.35)

    save(fig, "active_semantic_recovery_evidence")


def curate():
    stems = (
        "language_to_semantic_task_graph",
        "joint_task_viewpoint_path_optimization",
        "active_semantic_recovery_evidence",
    )
    SELECTED_OUT.mkdir(parents=True, exist_ok=True)
    for old in SELECTED_OUT.glob("*"):
        if old.is_file():
            old.unlink()
    for stem in stems:
        for suffix in (".png", ".pdf"):
            shutil.copy2(CANDIDATE_OUT / f"{stem}{suffix}", SELECTED_OUT / f"{stem}{suffix}")
    (SELECTED_OUT / "README.md").write_text(
        "# Stage-2 real-data paper assets\n\n"
        "All panels are rendered from stored 00337 task graphs, viewpoint candidates, "
        "3D A*/B-spline execution records, OWLv2 outputs, and Habitat RGB frames. "
        "No task, trajectory, detection, or recovery score is synthesized.\n",
        encoding="utf-8",
    )


def main():
    render_language_task_graph()
    render_joint_optimization()
    render_active_recovery()
    curate()
    print(f"generated candidates: {CANDIDATE_OUT}")
    print(f"curated assets:      {SELECTED_OUT}")


if __name__ == "__main__":
    main()
