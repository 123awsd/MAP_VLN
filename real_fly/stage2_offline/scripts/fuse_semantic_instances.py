#!/usr/bin/env python3
"""Balanced multi-frame semantic instance fusion for calibrated real RGB-D data."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np


ALIASES = {
    "monitor": "display", "television": "display",
    "table": "table", "dining table": "table", "desk": "table",
    "coffee table": "table", "side table": "table", "console table": "table",
    "chair": "seat", "armchair": "seat", "stool": "seat", "bench": "seat", "ottoman": "seat",
    "cabinet": "storage", "wardrobe": "storage", "dresser": "storage", "nightstand": "storage",
    "shelf": "shelf", "bookshelf": "shelf",
}


def canonical(label: str) -> str:
    return ALIASES.get(label, label)


def aabb_iou(a_center, a_size, b_center, b_size) -> float:
    a0, a1 = a_center - a_size / 2, a_center + a_size / 2
    b0, b1 = b_center - b_size / 2, b_center + b_size / 2
    overlap = np.maximum(0.0, np.minimum(a1, b1) - np.maximum(a0, b0))
    intersection = float(np.prod(overlap))
    union = float(np.prod(a_size) + np.prod(b_size) - intersection)
    return intersection / union if union > 1e-9 else 0.0


def weighted_quaternion(rows: list[dict]) -> list[float]:
    # Boxes have 180-degree yaw symmetry. Use the highest-quality orientation
    # after aligning quaternion signs; median dimensions provide the stability.
    best = max(rows, key=lambda row: row["quality"])
    return list(best["orientation_wxyz"])


def frame_sharpness(episode: Path) -> dict[int, float]:
    result = {}
    for path in sorted((episode / "frames/color").glob("*.jpg")):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is not None:
            result[int(path.stem)] = float(cv2.Laplacian(image, cv2.CV_64F).var())
    return result


def load_matched(args) -> tuple[list[dict], dict]:
    raw = json.loads(args.raw_targets.read_text(encoding="utf-8"))
    boxes_by_key = defaultdict(list)
    with args.boxer_3d.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            boxes_by_key[(int(row["time_ns"]), row["name"].strip().lower())].append(row)
    raw_by_key = defaultdict(list)
    for row in raw:
        raw_by_key[(int(row["time_ns"]), row["label"].strip().lower())].append(row)
    sharpness = frame_sharpness(args.episode)
    matched, rejected = [], Counter()
    for key, targets in raw_by_key.items():
        boxes = boxes_by_key.get(key, [])
        if not boxes:
            rejected["missing_3d_box"] += len(targets)
            continue
        target_centers = np.asarray([[r["world_x_m"], r["world_y_m"], r["world_z_m"]] for r in targets], float)
        box_centers = np.asarray([[r["tx_world_object"], r["ty_world_object"], r["tz_world_object"]] for r in boxes], float)
        costs = np.linalg.norm(target_centers[:, None, :] - box_centers[None, :, :], axis=2)
        assignments, used_targets, used_boxes = [], set(), set()
        for flat_index in np.argsort(costs, axis=None):
            ti, bi = np.unravel_index(flat_index, costs.shape)
            if int(ti) in used_targets or int(bi) in used_boxes:
                continue
            used_targets.add(int(ti)); used_boxes.add(int(bi)); assignments.append((int(ti), int(bi)))
            if len(assignments) == min(costs.shape):
                break
        for ti, bi in assignments:
            target, box = targets[int(ti)], boxes[int(bi)]
            match_distance = float(costs[ti, bi])
            if match_distance > args.max_geometry_match_distance:
                rejected["geometry_match_distance"] += 1
                continue
            p2d, p3d = float(target["confidence"]), float(box["prob"])
            if p2d < args.min_2d_confidence or p3d < args.min_3d_confidence:
                rejected["confidence"] += 1
                continue
            size = np.abs(np.asarray([box["scale_x"], box["scale_y"], box["scale_z"]], float))
            if np.any(size < args.min_dimension) or np.any(size > args.max_dimension):
                rejected["dimension"] += 1
                continue
            blur = sharpness.get(int(target["frame_index"]), args.full_sharpness)
            sharp_weight = min(1.0, max(args.min_sharpness_weight, blur / args.full_sharpness))
            quality = math.sqrt(p2d * p3d) * float(target["depth_valid_ratio"]) * sharp_weight
            matched.append({
                **target,
                "canonical_label": canonical(target["label"]),
                "center": target_centers[ti], "size": size,
                "orientation_wxyz": [float(box[k]) for k in
                                     ("qw_world_object", "qx_world_object", "qy_world_object", "qz_world_object")],
                "confidence_3d": p3d, "quality": quality,
                "sharpness": blur, "geometry_match_distance_m": match_distance,
            })
    return matched, dict(rejected)


def compatible(det: dict, cluster: list[dict], args) -> tuple[bool, float]:
    centers = np.asarray([row["center"] for row in cluster])
    center = np.median(centers, axis=0)
    size = np.median(np.asarray([row["size"] for row in cluster]), axis=0)
    distance = float(np.linalg.norm(det["center"] - center))
    overlap = aabb_iou(det["center"], det["size"], center, size)
    size_ratio = float(np.max(np.maximum(det["size"] / size, size / det["size"])))
    accepted = size_ratio <= args.max_size_ratio and (
        overlap >= args.strong_iou
        or distance <= args.strong_center_distance
        or (overlap >= args.weak_iou and distance <= args.weak_center_distance)
    )
    # Prefer high overlap and short distance when several clusters are valid.
    return accepted, distance - 0.75 * overlap


def fuse(detections: list[dict], args) -> list[dict]:
    clusters: list[list[dict]] = []
    for det in sorted(detections, key=lambda row: (-row["quality"], row["frame_index"])):
        candidates = []
        for index, cluster in enumerate(clusters):
            if cluster[0]["canonical_label"] != det["canonical_label"]:
                continue
            accepted, cost = compatible(det, cluster, args)
            if accepted:
                candidates.append((cost, index))
        if candidates:
            clusters[min(candidates)[1]].append(det)
        else:
            clusters.append([det])

    output = []
    for instance_id, rows in enumerate(clusters):
        unique_frames = len({row["frame_index"] for row in rows})
        if unique_frames < args.min_unique_frames:
            continue
        weights = np.asarray([max(1e-6, row["quality"]) for row in rows])
        centers = np.asarray([row["center"] for row in rows])
        center = np.average(centers, axis=0, weights=weights)
        distances = np.linalg.norm(centers - center, axis=1)
        sizes = np.asarray([row["size"] for row in rows])
        label_scores = defaultdict(float)
        for row in rows:
            label_scores[row["label"]] += row["quality"]
        label = max(label_scores, key=label_scores.get)
        orientation = weighted_quaternion(rows)
        output.append({
            "cluster_id": instance_id, "label": label,
            "canonical_label": rows[0]["canonical_label"],
            "world_x_m": float(center[0]), "world_y_m": float(center[1]), "world_z_m": float(center[2]),
            "size_x_m": float(np.median(sizes[:, 0])), "size_y_m": float(np.median(sizes[:, 1])),
            "size_z_m": float(np.median(sizes[:, 2])), "orientation_wxyz": orientation,
            "observations": len(rows), "unique_frames": unique_frames,
            "confidence_mean": float(np.mean([row["confidence"] for row in rows])),
            "confidence_max": float(np.max([row["confidence"] for row in rows])),
            "confidence_3d_mean": float(np.mean([row["confidence_3d"] for row in rows])),
            "depth_valid_ratio_mean": float(np.mean([row["depth_valid_ratio"] for row in rows])),
            "position_spread_p90_m": float(np.percentile(distances, 90)),
            "planning_eligible": True, "extrinsic_status": "accepted",
            "source_labels": dict(sorted(Counter(row["label"] for row in rows).items())),
            "member_detection_ids": [int(row["detection_id"]) for row in rows],
        })
    output.sort(key=lambda row: (row["label"], row["world_x_m"], row["world_y_m"]))
    for index, row in enumerate(output):
        row["cluster_id"] = index
    return output


def write_csv(path: Path, rows: list[dict]) -> None:
    fields = [key for key in rows[0] if key not in ("source_labels", "member_detection_ids", "orientation_wxyz")]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows({key: row[key] for key in fields} for row in rows)


def write_boxer_csv(path: Path, rows: list[dict]) -> None:
    fields = ["time_ns", "tx_world_object", "ty_world_object", "tz_world_object",
              "qw_world_object", "qx_world_object", "qy_world_object", "qz_world_object",
              "scale_x", "scale_y", "scale_z", "name", "instance", "sem_id", "prob"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for row in rows:
            qw, qx, qy, qz = row["orientation_wxyz"]
            writer.writerow({
                "time_ns": 0, "tx_world_object": row["world_x_m"],
                "ty_world_object": row["world_y_m"], "tz_world_object": row["world_z_m"],
                "qw_world_object": qw, "qx_world_object": qx, "qy_world_object": qy,
                "qz_world_object": qz, "scale_x": row["size_x_m"],
                "scale_y": row["size_y_m"], "scale_z": row["size_z_m"],
                "name": row["label"], "instance": row["cluster_id"], "sem_id": -1,
                "prob": row["confidence_max"],
            })


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--raw-targets", type=Path, required=True)
    parser.add_argument("--boxer-3d", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-2d-confidence", type=float, default=0.25)
    parser.add_argument("--min-3d-confidence", type=float, default=0.30)
    parser.add_argument("--min-unique-frames", type=int, default=2)
    parser.add_argument("--strong-center-distance", type=float, default=0.35)
    parser.add_argument("--weak-center-distance", type=float, default=0.60)
    parser.add_argument("--strong-iou", type=float, default=0.12)
    parser.add_argument("--weak-iou", type=float, default=0.03)
    parser.add_argument("--max-size-ratio", type=float, default=3.0)
    parser.add_argument("--min-dimension", type=float, default=0.04)
    parser.add_argument("--max-dimension", type=float, default=3.0)
    parser.add_argument("--max-geometry-match-distance", type=float, default=1.5)
    parser.add_argument("--full-sharpness", type=float, default=120.0)
    parser.add_argument("--min-sharpness-weight", type=float, default=0.35)
    args = parser.parse_args()
    detections, rejected = load_matched(args)
    instances = fuse(detections, args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "target_points_raw.json").write_text(
        json.dumps([{key: value for key, value in row.items() if key not in ("center", "size")}
                    for row in detections], indent=2, default=float) + "\n", encoding="utf-8")
    (args.output_dir / "target_points_clustered.json").write_text(
        json.dumps(instances, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_csv(args.output_dir / "target_points_clustered.csv", instances)
    write_boxer_csv(args.output_dir / "boxer_3dbbs_balanced_fused.csv", instances)
    report = {
        "format": "pre_map_vln.semantic_instance_fusion.v1", "status": "ok",
        "matched_filtered_detections": len(detections), "fused_instances": len(instances),
        "instances_by_class": dict(sorted(Counter(row["label"] for row in instances).items())),
        "rejected": rejected,
        "position_spread_median_m": float(np.median([row["position_spread_p90_m"] for row in instances])) if instances else None,
        "parameters": {key: value for key, value in vars(args).items() if not isinstance(value, Path)},
    }
    (args.output_dir / "fusion_report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
