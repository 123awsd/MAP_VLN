#!/usr/bin/env python3
"""Survey local MP3D scenes before expensive benchmark generation."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import habitat_sim
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ROOM_SKIP = {"stairs", "hallway", "entryway/foyer/lobby", "balcony", "outdoor"}
TARGET_LABELS = {
    "bed", "chair", "table", "sofa", "couch", "television", "tv", "lamp",
    "cabinet", "shelving", "refrigerator", "microwave", "sink", "toilet",
    "desk", "book", "picture", "plant", "towel", "clothes", "objects",
}


def vec(value) -> list[float]:
    return [float(item) for item in value]


def floor_clusters(values: np.ndarray, tolerance: float = 0.35) -> list[dict]:
    clusters: list[list[float]] = []
    for value in sorted(float(item) for item in values):
        if not clusters or value - float(np.mean(clusters[-1])) > tolerance:
            clusters.append([value])
        else:
            clusters[-1].append(value)
    total = max(1, len(values))
    return [
        {
            "height_m": round(float(np.median(cluster)), 4),
            "samples": len(cluster),
            "fraction": round(len(cluster) / total, 4),
        }
        for cluster in sorted(clusters, key=len, reverse=True)
    ]


def survey(scene_path: Path, samples: int, seed: int, scene_config: Path | None = None) -> dict:
    cfg = habitat_sim.SimulatorConfiguration()
    cfg.scene_id = str(scene_path.resolve())
    cfg.create_renderer = False
    if scene_config is not None:
        cfg.scene_dataset_config_file = str(scene_config.resolve())
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    with habitat_sim.Simulator(habitat_sim.Configuration(cfg, [agent_cfg])) as sim:
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("navmesh did not load")
        sim.pathfinder.seed(seed)
        points = np.asarray(
            [sim.pathfinder.get_random_navigable_point() for _ in range(samples)],
            dtype=np.float64,
        )
        floors = floor_clusters(points[:, 1])
        categories = Counter(
            obj.category.name().strip().lower()
            for obj in sim.semantic_scene.objects if obj is not None and obj.category is not None
        )
        rooms = [
            {
                "id": region.id,
                "label": (
                    region.category.name().strip().lower()
                    if region.category is not None and region.category.name() else "unknown"
                ),
                "center_xyz_m": vec(region.aabb.center()),
                "size_xyz_m": vec(region.aabb.size()),
                "object_count": len(region.objects),
            }
            for region in sim.semantic_scene.regions
        ]
        destination_rooms = [room for room in rooms if room["label"] not in ROOM_SKIP]
        target_instances = sum(categories[label] for label in TARGET_LABELS)
        dominant_fraction = floors[0]["fraction"] if floors else 0.0
        score = (
            3.0 * min(len(destination_rooms), 12)
            + min(target_instances, 80)
            + 30.0 * dominant_fraction
        )
        return {
            "scene_id": scene_path.parent.name,
            "scene_path": str(scene_path.resolve()),
            "navmesh_area_m2": round(float(sim.pathfinder.navigable_area), 3),
            "bounds_min_xyz_m": vec(sim.pathfinder.get_bounds()[0]),
            "bounds_max_xyz_m": vec(sim.pathfinder.get_bounds()[1]),
            "floor_clusters": floors,
            "region_count": len(rooms),
            "destination_room_count": len(destination_rooms),
            "regions": rooms,
            "semantic_object_count": sum(categories.values()),
            "target_instance_count": target_instances,
            "top_categories": dict(categories.most_common(40)),
            "selection_score": round(score, 3),
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-root", type=Path, default=ROOT / "data/scene_datasets/mp3d")
    parser.add_argument("--scene-config", type=Path)
    parser.add_argument("--scene-ids", nargs="*")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=1500)
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    scenes = sorted(args.scene_root.glob("*/*.basis.glb"))
    if not scenes:
        scenes = sorted(args.scene_root.glob("*/*.glb"))
    if args.scene_ids:
        selected = set(args.scene_ids)
        scenes = [scene for scene in scenes if scene.parent.name in selected]
    if not scenes:
        raise RuntimeError(f"no MP3D scenes below {args.scene_root}")
    records = []
    for scene in scenes:
        record = survey(scene, args.samples, args.seed, args.scene_config)
        records.append(record)
        print(
            f"{record['scene_id']}: area={record['navmesh_area_m2']:.1f} "
            f"rooms={record['destination_room_count']} targets={record['target_instance_count']} "
            f"dominant_floor={record['floor_clusters'][0]['fraction']:.2f}",
            flush=True,
        )
    ranked = sorted(records, key=lambda item: item["selection_score"], reverse=True)
    payload = {
        "format": "pre_map_vln.scene_survey.v1",
        "scene_count": len(records),
        "recommended_order": [item["scene_id"] for item in ranked],
        "scenes": records,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
