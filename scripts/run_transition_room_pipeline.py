#!/usr/bin/env python3
"""Run transition-aware per-floor room segmentation without changing OccuSG.

The script creates auditable hard/soft/uncertain transition masks, runs OccuSG
on geometry-only wall grids, and annotates the returned regions afterwards.
Raw OccuSG outputs are copied unchanged next to the postprocessed results.
"""
from __future__ import annotations

import argparse
import copy
import json
import shutil
import subprocess
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon


ROOT = Path(__file__).resolve().parents[1]


def world_pixels(points, metadata):
    points = np.asarray(points, dtype=np.float64)
    origin = np.asarray(metadata["origin_xy_m"], dtype=np.float64)
    resolution = float(metadata["resolution_m"])
    return np.rint((points[:, :2] - origin) / resolution).astype(np.int32)


def clip_pixels(points, shape):
    points = points.copy()
    points[:, 0] = np.clip(points[:, 0], 0, shape[1] - 1)
    points[:, 1] = np.clip(points[:, 1], 0, shape[0] - 1)
    return points


def transition_masks(grid, metadata, transitions, floor):
    hard = np.zeros(grid.shape, dtype=np.uint8)
    soft = np.zeros(grid.shape, dtype=np.uint8)
    uncertain = np.zeros(grid.shape, dtype=np.uint8)
    transition_ids = np.zeros(grid.shape, dtype=np.int16)
    resolution = float(metadata["resolution_m"])
    hard_width = max(3, int(round(0.70 / resolution)))
    uncertain_width = max(2, int(round(0.18 / resolution)))
    entrance_radius = max(2, int(round(0.45 / resolution)))
    included = []
    for label, transition in enumerate(transitions, start=1):
        connected = [int(value) for value in transition["connected_floors"]]
        if floor not in connected:
            continue
        included.append((label, transition))
        local_hard = np.zeros_like(hard)
        local_soft = np.zeros_like(soft)
        for traversal in transition.get("traversals", []):
            line = world_pixels(traversal["centerline_xyz"], metadata)
            line = clip_pixels(line, grid.shape)
            if len(line) >= 2:
                cv2.polylines(local_hard, [line], False, 1, thickness=hard_width)
        for key in ("lower_entrance_xyz", "upper_entrance_xyz"):
            center = world_pixels([transition[key]], metadata)[0]
            if 0 <= center[0] < grid.shape[1] and 0 <= center[1] < grid.shape[0]:
                cv2.circle(local_hard, tuple(center), entrance_radius, 1, thickness=-1)

        sections = transition.get("boundary_sections", [])
        if sections:
            left = np.asarray([item["left_xyz"] for item in sections])
            right = np.asarray([item["right_xyz"] for item in sections])
            polygon = world_pixels(np.vstack((left, right[::-1])), metadata)
            polygon = clip_pixels(polygon, grid.shape)
            if len(polygon) >= 3:
                cv2.fillPoly(local_soft, [polygon], 1)
            for side in ("left", "right"):
                key = f"{side}_xyz"
                confidence = f"{side}_observed_wall"
                for first, second in zip(sections, sections[1:]):
                    if first.get(confidence) and second.get(confidence):
                        continue
                    segment = world_pixels([first[key], second[key]], metadata)
                    segment = clip_pixels(segment, grid.shape)
                    cv2.line(uncertain, tuple(segment[0]), tuple(segment[1]), 1, uncertain_width)
            for section, score_name in (
                (sections[0], "lower_floor_opening_score"),
                (sections[-1], "upper_floor_opening_score"),
            ):
                if float(transition.get(score_name, 0.0)) >= 0.15:
                    continue
                segment = world_pixels([section["left_xyz"], section["right_xyz"]], metadata)
                segment = clip_pixels(segment, grid.shape)
                cv2.line(uncertain, tuple(segment[0]), tuple(segment[1]), 1, uncertain_width)
        local_soft |= local_hard
        hard |= local_hard
        soft |= local_soft
        transition_ids[(local_soft > 0) & (transition_ids == 0)] = label
    return hard.astype(bool), soft.astype(bool), uncertain.astype(bool), transition_ids, included


def polygon_mask(polygon, metadata, shape):
    result = np.zeros(shape, dtype=np.uint8)
    pixels = world_pixels(polygon, metadata)
    pixels = clip_pixels(pixels, shape)
    if len(pixels) >= 3:
        cv2.fillPoly(result, [pixels], 1)
    return result.astype(bool)


def mask_polygon(mask, metadata):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
    origin = np.asarray(metadata["origin_xy_m"], dtype=np.float64)
    resolution = float(metadata["resolution_m"])
    return (origin + contour * resolution).tolist()


def annotate_regions(raw, grid, metadata, hard, soft, transition_ids, included):
    regions = copy.deepcopy(raw.get("regions", []))
    resolution = float(metadata["resolution_m"])
    transition_by_label = {label: transition for label, transition in included}
    members = {label: [] for label, _ in included}
    for region in regions:
        region_mask = polygon_mask(region["polygon_xy_m"], metadata, grid.shape)
        cells = max(1, int(region_mask.sum()))
        best = None
        for label, transition in included:
            id_mask = transition_ids == label
            hard_overlap = int(np.count_nonzero(region_mask & hard & id_mask)) / cells
            soft_overlap = int(np.count_nonzero(region_mask & soft & id_mask)) / cells
            centroid = world_pixels([region["centroid_xy_m"]], metadata)[0]
            centroid_inside = (
                0 <= centroid[0] < grid.shape[1] and 0 <= centroid[1] < grid.shape[0]
                and bool(id_mask[centroid[1], centroid[0]])
            )
            soft_area = float(np.count_nonzero(id_mask) * resolution**2)
            area_limit = max(5.0, 2.5 * soft_area)
            score = 0.55 * hard_overlap + 0.35 * soft_overlap + 0.10 * centroid_inside
            eligible = float(region["area_m2"]) <= area_limit
            matched = eligible and (
                hard_overlap >= 0.10 or soft_overlap >= 0.35 or centroid_inside
            )
            candidate = (score, label, hard_overlap, soft_overlap, centroid_inside, matched)
            if best is None or candidate[0] > best[0]:
                best = candidate
        region["space_role"] = "room"
        region["transition_match"] = None
        if best and best[-1]:
            score, label, hard_overlap, soft_overlap, centroid_inside, _ = best
            transition = transition_by_label[label]
            region["space_role"] = "transition_space"
            region["transition_match"] = {
                "transition_id": transition["id"],
                "score": float(score),
                "hard_overlap_ratio": float(hard_overlap),
                "soft_overlap_ratio": float(soft_overlap),
                "centroid_inside": bool(centroid_inside),
            }
            members[label].append(int(region["id"]))

    transition_spaces = []
    for label, transition in included:
        local = transition_ids == label
        transition_spaces.append({
            "id": transition["id"],
            "space_role": "transition_space",
            "connected_floors": transition["connected_floors"],
            "classification": transition["classification"],
            "confidence": transition["confidence"],
            "boundary_closed": transition["boundary_closed"],
            "source_region_ids": members[label],
            "source": "occusg_overlap" if members[label] else "transition_mask_only",
            "area_m2": float(np.count_nonzero(local) * resolution**2),
            "polygon_xy_m": mask_polygon(local, metadata),
        })
    return {
        "format": "pre_map_vln.transition_aware_regions.v1",
        "source_occusg_format": raw.get("format"),
        "raw_region_count": len(regions),
        "ordinary_region_count": sum(region["space_role"] == "room" for region in regions),
        "transition_region_count": sum(region["space_role"] == "transition_space" for region in regions),
        "regions": regions,
        "transition_spaces": transition_spaces,
    }


def grid_rgb(grid):
    image = np.zeros((*grid.shape, 3), dtype=np.uint8)
    image[grid < 0] = (165, 165, 165)
    image[grid == 0] = (248, 248, 248)
    image[grid == 100] = (28, 28, 28)
    return image


def extent_for(grid, metadata):
    origin = np.asarray(metadata["origin_xy_m"])
    resolution = float(metadata["resolution_m"])
    return [origin[0], origin[0] + grid.shape[1] * resolution,
            origin[1], origin[1] + grid.shape[0] * resolution]


def draw_regions(axis, regions, transition_only=False):
    palette = plt.get_cmap("tab20")
    for index, region in enumerate(regions):
        is_transition = region.get("space_role") == "transition_space"
        if transition_only and not is_transition:
            continue
        polygon = np.asarray(region.get("polygon_xy_m", []), dtype=np.float64)
        if len(polygon) < 3:
            continue
        color = "#ff7a00" if is_transition else palette(index % 20)
        axis.add_patch(Polygon(polygon, closed=True, facecolor=color, edgecolor="#202020", alpha=.38, linewidth=1.0))
        center = region["centroid_xy_m"]
        axis.text(center[0], center[1], f'R{region["id"]}', fontsize=7, ha="center", va="center")


def render_floor_diagnostic(path, floor, grid, metadata, hard, soft, uncertain, raw, post):
    extent = extent_for(grid, metadata)
    figure, axes = plt.subplots(1, 5, figsize=(28, 6), sharex=True, sharey=True)
    base = grid_rgb(grid)
    titles = ["Geometry wall grid", "Hard transition mask", "Soft + uncertain masks",
              "Raw OccuSG regions", "Transition-aware result"]
    for axis, title in zip(axes, titles):
        axis.imshow(base, origin="lower", extent=extent, interpolation="nearest")
        axis.set_title(f"L{floor} | {title}")
        axis.set_aspect("equal"); axis.grid(alpha=.10); axis.set_xlabel("FALCON X (m)")
    axes[0].set_ylabel("FALCON Y (m)")
    for axis, mask, color, alpha in (
        (axes[1], hard, "#e600ff", .58),
        (axes[2], soft, "#ff9800", .38),
        (axes[2], uncertain, "#00bcd4", .85),
    ):
        overlay = np.zeros((*mask.shape, 4), dtype=np.float32)
        rgb = np.asarray(plt.matplotlib.colors.to_rgb(color))
        overlay[mask, :3] = rgb; overlay[mask, 3] = alpha
        axis.imshow(overlay, origin="lower", extent=extent, interpolation="nearest")
    draw_regions(axes[3], raw.get("regions", []))
    draw_regions(axes[4], post.get("regions", []))
    for transition in post.get("transition_spaces", []):
        polygon = np.asarray(transition.get("polygon_xy_m", []))
        if len(polygon) >= 3:
            axes[4].add_patch(Polygon(polygon, closed=True, fill=False, edgecolor="#e600ff", linewidth=2.5, linestyle="--"))
    figure.tight_layout()
    figure.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("wall_grid_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-floor", type=int, default=3)
    parser.add_argument("--decomp-threshold", type=float, default=1.8)
    parser.add_argument(
        "--occusg-output-group", default="",
        help="optional path below outputs/occusg used to group one pipeline run",
    )
    args = parser.parse_args()
    transition_dir = args.run_dir / "transition_reconstruction"
    transition_json = transition_dir / "transitions.json"
    subprocess.run([
        str(ROOT / ".envs/habitat/bin/python"),
        str(ROOT / "scripts/reconstruct_transition_spaces.py"),
        str(args.run_dir), str(transition_dir), "--max-floor", str(args.max_floor),
    ], check=True)
    transitions = json.loads(transition_json.read_text())["transitions"]
    args.output.mkdir(parents=True, exist_ok=True)
    summary = []
    figures = []
    for floor in range(1, args.max_floor + 1):
        source = args.wall_grid_dir / f"L{floor}_wall_grid"
        grid = np.load(source.with_suffix(".npy"))
        metadata = json.loads(source.with_suffix(".json").read_text())
        hard, soft, uncertain, transition_ids, included = transition_masks(
            grid, metadata, transitions, floor
        )
        prefix = args.output / f"L{floor}_transition"
        np.save(prefix.with_name(prefix.name + "_hard.npy"), hard)
        np.save(prefix.with_name(prefix.name + "_soft.npy"), soft)
        np.save(prefix.with_name(prefix.name + "_uncertain.npy"), uncertain)
        np.save(prefix.with_name(prefix.name + "_ids.npy"), transition_ids)
        mask_metadata = {
            "format": "pre_map_vln.transition_masks.v1", "floor": floor,
            "resolution_m": metadata["resolution_m"], "origin_xy_m": metadata["origin_xy_m"],
            "hard_cell_count": int(hard.sum()), "soft_cell_count": int(soft.sum()),
            "uncertain_cell_count": int(uncertain.sum()),
            "transition_ids": {str(label): transition["id"] for label, transition in included},
        }
        prefix.with_name(prefix.name + "_masks.json").write_text(json.dumps(mask_metadata, indent=2) + "\n")

        run_name = f"{args.output.name}_L{floor}_geometry"
        runtime_prefix = ROOT / "runtime/occusg" / f"{run_name}_grid"
        shutil.copy2(source.with_suffix(".npy"), runtime_prefix.with_suffix(".npy"))
        shutil.copy2(source.with_suffix(".json"), runtime_prefix.with_suffix(".json"))
        output_rel = (
            f"{args.occusg_output_group}/L{floor}_geometry"
            if args.occusg_output_group else run_name
        )
        subprocess.run([
            str(ROOT / "scripts/run_occusg.sh"), run_name,
            str(args.decomp_threshold), output_rel,
        ], check=True)
        raw_path = ROOT / "outputs/occusg" / output_rel / "regions.json"
        raw = json.loads(raw_path.read_text())
        raw_copy = args.output / f"L{floor}_regions_raw.json"
        shutil.copy2(raw_path, raw_copy)
        post = annotate_regions(raw, grid, metadata, hard, soft, transition_ids, included)
        post_path = args.output / f"L{floor}_regions_transition_aware.json"
        post_path.write_text(json.dumps(post, indent=2) + "\n")
        figure = args.output / f"L{floor}_transition_room_diagnostic.png"
        render_floor_diagnostic(figure, floor, grid, metadata, hard, soft, uncertain, raw, post)
        figures.append(str(figure))
        summary.append({
            "floor": floor, "raw_regions": len(raw.get("regions", [])),
            "ordinary_regions": post["ordinary_region_count"],
            "transition_regions": post["transition_region_count"],
            "transition_spaces": len(post["transition_spaces"]),
            "hard_area_m2": float(hard.sum() * float(metadata["resolution_m"])**2),
            "soft_area_m2": float(soft.sum() * float(metadata["resolution_m"])**2),
        })
    document = {
        "format": "pre_map_vln.transition_room_pipeline.v1",
        "source_run": str(args.run_dir.resolve()), "source_wall_grids": str(args.wall_grid_dir.resolve()),
        "occusg_decomp_threshold": args.decomp_threshold, "floors": summary, "figures": figures,
    }
    if figures:
        figure, axes = plt.subplots(len(figures), 1, figsize=(24, 5.3 * len(figures)), squeeze=False)
        for axis, image_path, item in zip(axes[:, 0], figures, summary):
            axis.imshow(plt.imread(image_path))
            axis.axis("off")
            axis.set_title(
                f'L{item["floor"]}: {item["raw_regions"]} raw → '
                f'{item["ordinary_regions"]} rooms + {item["transition_regions"]} transition regions',
                fontsize=13,
            )
        figure.tight_layout()
        summary_figure = args.output / "all_floors_transition_room_summary.png"
        figure.savefig(summary_figure, dpi=130, bbox_inches="tight")
        plt.close(figure)
        document["summary_figure"] = str(summary_figure)
    (args.output / "summary.json").write_text(json.dumps(document, indent=2) + "\n")
    print(json.dumps(document, indent=2))


if __name__ == "__main__":
    main()
