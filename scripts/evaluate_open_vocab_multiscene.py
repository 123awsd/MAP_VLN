#!/usr/bin/env python3
"""Render three real HM3D scenes and benchmark local OWLv2 perception."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

import habitat_sim
import numpy as np
from habitat_sim.utils.common import quat_from_angle_axis
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from stage2.io_utils import atomic_json  # noqa: E402
from stage2.open_vocab_detector import LocalOpenVocabularyDetector  # noqa: E402


PROMPTS = ["chair", "table", "sofa", "bed", "lamp", "cabinet", "door", "television", "plant", "shelf"]
ALIASES = {
    "television": {"television", "tv", "monitor"},
    "sofa": {"sofa", "couch"},
    "lamp": {"lamp", "ceiling lamp", "floor lamp", "table lamp"},
    "plant": {"plant", "indoor plant", "potted plant"},
    "shelf": {"shelf", "shelving", "bookshelf"},
}
SCENES = {
    "00337-CFVBbU9Rsyb": "CFVBbU9Rsyb",
    "00770-NBg5UqG3di3": "NBg5UqG3di3",
    "00861-GLAQ4DNUx5U": "GLAQ4DNUx5U",
}
ANNOTATED_SCENE = "00861-GLAQ4DNUx5U"


def camera(uuid: str, sensor_type: habitat_sim.SensorType) -> habitat_sim.CameraSensorSpec:
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid, spec.sensor_type = uuid, sensor_type
    spec.resolution = [480, 640]
    spec.position = [0.0, 1.0, 0.0]
    spec.hfov = 90.0
    return spec


def semantic_presence(sim, semantic: np.ndarray, minimum_pixels: int) -> tuple[dict[str, bool], dict[str, int]]:
    category_counts: dict[str, int] = {}
    ids, counts = np.unique(semantic, return_counts=True)
    objects = sim.semantic_scene.objects
    for semantic_id, count in zip(ids.tolist(), counts.tolist()):
        if 0 <= semantic_id < len(objects) and objects[semantic_id] is not None:
            label = str(objects[semantic_id].category.name()).strip().lower()
            category_counts[label] = category_counts.get(label, 0) + int(count)
    presence = {}
    for prompt in PROMPTS:
        accepted = ALIASES.get(prompt, {prompt})
        pixels = sum(
            count for label, count in category_counts.items()
            if label in accepted or any(alias in label for alias in accepted)
        )
        presence[prompt] = pixels >= minimum_pixels
    return presence, category_counts


def render_dataset(output_dir: Path, views_per_scene: int, minimum_pixels: int, seed: int) -> list[dict]:
    records = []
    for scene_index, (scene_id, stem) in enumerate(SCENES.items()):
        scene_dir = ROOT / "data/scene_datasets/hm3d/example" / scene_id
        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_id = str(scene_dir / f"{stem}.basis.glb")
        sim_cfg.scene_dataset_config_file = str(
            ROOT / "data/scene_datasets/hm3d/example" /
            ("hm3d_annotated_example_basis.scene_dataset_config.json" if scene_id == ANNOTATED_SCENE else "hm3d_example_basis.scene_dataset_config.json")
        )
        specs = [camera("rgb", habitat_sim.SensorType.COLOR)]
        if scene_id == ANNOTATED_SCENE:
            specs.append(camera("semantic", habitat_sim.SensorType.SEMANTIC))
        agent_cfg = habitat_sim.agent.AgentConfiguration()
        agent_cfg.sensor_specifications = specs
        frame_dir = output_dir / "frames" / scene_id
        frame_dir.mkdir(parents=True, exist_ok=True)
        with habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg])) as sim:
            if not sim.pathfinder.is_loaded:
                raise RuntimeError(f"navmesh failed for {scene_id}")
            sim.pathfinder.seed(seed + scene_index)
            agent = sim.initialize_agent(0)
            for view_index in range(views_per_scene):
                position = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float64)
                yaw = 2.0 * math.pi * ((view_index * 0.61803398875) % 1.0)
                state = habitat_sim.AgentState()
                state.position = position
                state.rotation = quat_from_angle_axis(yaw, np.asarray([0.0, 1.0, 0.0]))
                agent.set_state(state)
                observations = sim.get_sensor_observations()
                path = frame_dir / f"view_{view_index:04d}.jpg"
                Image.fromarray(np.asarray(observations["rgb"])[..., :3].astype(np.uint8)).save(path, quality=90)
                record = {
                    "scene": scene_id, "view_index": view_index,
                    "image": str(path.relative_to(output_dir)),
                    "position_habitat": position.tolist(), "yaw_rad": yaw,
                    "has_semantic_gt": scene_id == ANNOTATED_SCENE,
                }
                if scene_id == ANNOTATED_SCENE:
                    presence, counts = semantic_presence(sim, np.asarray(observations["semantic"]), minimum_pixels)
                    record["gt_presence"] = presence
                    record["semantic_category_pixels"] = counts
                records.append(record)
    return records


def safe_ratio(numerator: int, denominator: int) -> float | None:
    return None if denominator == 0 else numerator / denominator


def classification_counts(records: list[dict], prompt: str | None, threshold: float) -> dict:
    counts = {key: 0 for key in ("tp", "fp", "fn", "tn")}
    prompts = [prompt] if prompt is not None else PROMPTS
    for item in records:
        for label in prompts:
            truth = bool(item["gt_presence"][label])
            prediction = any(
                detected["label"] == label and float(detected["score"]) >= threshold
                for detected in item["owlv2"]["detections"]
            )
            key = "tp" if truth and prediction else "fp" if prediction else "fn" if truth else "tn"
            counts[key] += 1
    counts.update({
        "precision": safe_ratio(counts["tp"], counts["tp"] + counts["fp"]),
        "recall": safe_ratio(counts["tp"], counts["tp"] + counts["fn"]),
        "f1": safe_ratio(2 * counts["tp"], 2 * counts["tp"] + counts["fp"] + counts["fn"]),
    })
    return counts


def evaluate(records: list[dict], output_dir: Path, threshold: float) -> dict:
    started = time.monotonic()
    with LocalOpenVocabularyDetector(PROMPTS, threshold=threshold, timeout_s=300.0) as detector:
        startup = detector.ready
        for index, record in enumerate(records):
            image_path = output_dir / record["image"]
            result = detector.detect(image_path)
            predicted = {item["label"] for item in result["detections"]}
            record["owlv2"] = {
                "latency_ms": result["latency_ms"], "predicted_labels": sorted(predicted),
                "detections": result["detections"],
            }
            if index < 12:
                canvas = Image.open(image_path).convert("RGB")
                draw = ImageDraw.Draw(canvas)
                for item in result["detections"]:
                    draw.rectangle(item["bbox_xyxy"], outline=(15, 75, 180), width=3)
                    draw.text((item["bbox_xyxy"][0], max(0, item["bbox_xyxy"][1] - 14)), f"{item['label']} {item['score']:.2f}", fill=(15, 75, 180))
                overlay = output_dir / "overlays" / record["scene"] / Path(record["image"]).name
                overlay.parent.mkdir(parents=True, exist_ok=True)
                canvas.save(overlay, quality=95)

    annotated = [item for item in records if item["has_semantic_gt"]]
    per_class = {}
    totals = classification_counts(annotated, None, threshold)
    sweep_thresholds = [round(0.20 + 0.025 * index, 3) for index in range(17)]
    threshold_sweep = [
        {"threshold": value, **classification_counts(annotated, None, value)}
        for value in sweep_thresholds
    ]
    recommended = max(
        threshold_sweep,
        key=lambda item: (item["f1"] if item["f1"] is not None else -1.0, item["precision"] or 0.0),
    )
    for prompt in PROMPTS:
        counts = classification_counts(annotated, prompt, threshold)
        class_sweep = [
            {"threshold": value, **classification_counts(annotated, prompt, value)}
            for value in sweep_thresholds
        ]
        best = max(class_sweep, key=lambda item: (item["f1"] if item["f1"] is not None else -1.0, item["precision"] or 0.0))
        counts.update({
            "positive_gt_frames": counts["tp"] + counts["fn"],
            "best_threshold": best["threshold"], "best_f1": best["f1"],
        })
        per_class[prompt] = counts
    latencies = [item["owlv2"]["latency_ms"] for item in records]
    scene_summary = []
    for scene in SCENES:
        values = [item for item in records if item["scene"] == scene]
        labels = sorted({label for item in values for label in item["owlv2"]["predicted_labels"]})
        scene_summary.append({
            "scene": scene, "view_count": len(values),
            "has_semantic_gt": scene == ANNOTATED_SCENE,
            "mean_detections_per_frame": statistics.mean(len(item["owlv2"]["detections"]) for item in values),
            "detected_label_coverage": labels,
        })
    return {
        "format": "pre_map_vln.open_vocab_multiscene.v1", "status": "passed",
        "protocol": {
            "scenes": list(SCENES), "views_per_scene": len(records) // len(SCENES),
            "prompt_count": len(PROMPTS), "prompts": PROMPTS, "threshold": threshold,
            "quantitative_gt_scenes": [ANNOTATED_SCENE],
            "boundary": "00337 and 00770 have real RGB/navmesh but no semantic mesh; only latency, detections and coverage are reported for them.",
        },
        "startup": startup,
        "latency_ms": {"mean": statistics.mean(latencies), "median": statistics.median(latencies), "p95": float(np.percentile(latencies, 95)), "max": max(latencies)},
        "image_level_semantic_gt": {
            "micro": totals,
            "recommended_global_threshold": recommended["threshold"],
            "recommended_global_metrics": recommended,
            "threshold_sweep": threshold_sweep,
            "per_class": per_class,
        },
        "scene_summary": scene_summary,
        "elapsed_s": time.monotonic() - started,
        "records": records,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/stage2/open_vocab_multiscene")
    parser.add_argument("--views-per-scene", type=int, default=48)
    parser.add_argument("--semantic-minimum-pixels", type=int, default=80)
    parser.add_argument("--threshold", type=float, default=0.20)
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = render_dataset(args.output_dir, args.views_per_scene, args.semantic_minimum_pixels, args.seed)
    report = evaluate(records, args.output_dir, args.threshold)
    atomic_json(args.output_dir / "report.json", report)
    print(json.dumps({key: report[key] for key in ("status", "protocol", "startup", "latency_ms", "image_level_semantic_gt", "scene_summary", "elapsed_s")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
