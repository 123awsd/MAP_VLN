#!/usr/bin/env python3
"""Export MP3D semantic instance labels and held-out ground truth in FALCON coordinates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import habitat_sim
import numpy as np


S_HABITAT_TO_FALCON = np.asarray(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


def values(vector) -> list[float]:
    return [round(float(value), 6) for value in vector]


def local_point(point, origin) -> list[float]:
    return values(S_HABITAT_TO_FALCON @ (np.asarray(point, dtype=np.float64) - origin))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--semantic-labels", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    args = parser.parse_args()

    manifest = json.loads(args.episode_manifest.read_text(encoding="utf-8"))
    origin = np.asarray(manifest["habitat_agent_origin_xyz_m"], dtype=np.float64)
    cfg = habitat_sim.SimulatorConfiguration()
    cfg.scene_id = str(args.scene.expanduser().resolve())
    cfg.create_renderer = False
    if args.scene_config is not None:
        cfg.scene_dataset_config_file = str(args.scene_config.expanduser().resolve())
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    with habitat_sim.Simulator(habitat_sim.Configuration(cfg, [agent_cfg])) as sim:
        objects = []
        labels = []
        for semantic_id, obj in enumerate(sim.semantic_scene.objects):
            if obj is None or obj.category is None:
                continue
            label = obj.category.name().strip().lower()
            labels.append((semantic_id, obj.id, label))
            center_h = np.asarray(obj.obb.center, dtype=np.float64)
            # S only permutes/sign-flips axes, so extents map by absolute S.
            sizes_f = np.abs(S_HABITAT_TO_FALCON) @ np.asarray(obj.obb.sizes, dtype=np.float64)
            objects.append({
                "semantic_id": semantic_id,
                "id": obj.id,
                "label": label,
                "center_xyz_m": local_point(center_h, origin),
                "size_xyz_m": values(sizes_f),
                "region_id": None if obj.region is None else obj.region.id,
            })
        regions = []
        for region in sim.semantic_scene.regions:
            center_h = np.asarray(region.aabb.center(), dtype=np.float64)
            size_h = np.asarray(region.aabb.size(), dtype=np.float64)
            regions.append({
                "id": region.id,
                "label": (
                    region.category.name().strip().lower()
                    if region.category is not None and region.category.name() else "unknown"
                ),
                "center_xyz_m": local_point(center_h, origin),
                "size_xyz_m": values(np.abs(S_HABITAT_TO_FALCON) @ size_h),
                "object_ids": [obj.id for obj in region.objects if obj is not None],
            })

    args.semantic_labels.parent.mkdir(parents=True, exist_ok=True)
    with args.semantic_labels.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerows(labels)
    payload = {
        "format": "pre_map_vln.mp3d_ground_truth.v1",
        "scene_id": manifest["scene"],
        "coordinate_frame": manifest["coordinate_frame"],
        "source": "official Matterport semantic annotations; evaluation only",
        "regions": regions,
        "objects": objects,
    }
    args.ground_truth.parent.mkdir(parents=True, exist_ok=True)
    args.ground_truth.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({
        "scene": manifest["scene"],
        "semantic_label_count": len(labels),
        "ground_truth_object_count": len(objects),
        "ground_truth_region_count": len(regions),
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
