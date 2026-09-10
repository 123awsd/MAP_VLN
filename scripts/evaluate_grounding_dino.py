#!/usr/bin/env python3
"""Run a small local Grounding DINO open-vocabulary smoke test.

This is intentionally separate from the existing OWLv2 backend.  It produces
annotated RGB frames and a JSON report so the two detectors can be compared
before changing the semantic pipeline.
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import torch
from PIL import Image, ImageDraw
from transformers import GroundingDinoForObjectDetection, GroundingDinoProcessor


DEFAULT_PROMPTS = [
    "a whiteboard",
    "a water dispenser",
    "a fire extinguisher",
    "a television cabinet",
    "a speaker",
    "a door",
    "a table",
    "a chair",
]


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--frames-dir",
        type=Path,
        default=root / "real_fly/stage2_offline/data/test_room/episode/frames/color",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / "real_fly/stage2_offline/data/test_room/grounding_dino_smoke",
    )
    parser.add_argument("--model", default="IDEA-Research/grounding-dino-tiny")
    parser.add_argument("--prompt", nargs="+", default=DEFAULT_PROMPTS)
    parser.add_argument("--max-images", type=int, default=20)
    parser.add_argument("--box-threshold", type=float, default=0.25)
    parser.add_argument("--text-threshold", type=float, default=0.20)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


def normalize_prompt(prompt: str) -> str:
    return prompt.strip().lower().rstrip(".")


def main() -> None:
    args = parse_args()
    if args.max_images <= 0:
        raise SystemExit("--max-images must be positive")
    images = sorted(args.frames_dir.glob("*.jpg")) + sorted(args.frames_dir.glob("*.png"))
    images = images[: args.max_images]
    if not images:
        raise FileNotFoundError(f"no RGB frames under {args.frames_dir}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    prompts = [normalize_prompt(item) for item in args.prompt if normalize_prompt(item)]
    if not prompts:
        raise SystemExit("at least one non-empty --prompt is required")

    device = torch.device(args.device)
    print(f"Loading {args.model} on {device} ...", flush=True)
    started = time.perf_counter()
    processor = GroundingDinoProcessor.from_pretrained(args.model)
    model = GroundingDinoForObjectDetection.from_pretrained(args.model).to(device)
    model.eval()
    print(f"Model ready in {time.perf_counter() - started:.1f}s", flush=True)

    records = []
    for index, path in enumerate(images, start=1):
        image = Image.open(path).convert("RGB")
        inputs = processor(images=image, text=[prompts], return_tensors="pt")
        inputs = {key: value.to(device) if hasattr(value, "to") else value for key, value in inputs.items()}
        infer_started = time.perf_counter()
        with torch.inference_mode():
            outputs = model(**inputs)
        target_sizes = torch.tensor([image.size[::-1]], device=device)
        result = processor.post_process_grounded_object_detection(
            outputs,
            inputs["input_ids"],
            threshold=args.box_threshold,
            text_threshold=args.text_threshold,
            target_sizes=target_sizes,
        )[0]
        latency_ms = (time.perf_counter() - infer_started) * 1000.0

        detections = []
        canvas = image.copy()
        draw = ImageDraw.Draw(canvas)
        for box, score, label in zip(result["boxes"], result["scores"], result["text_labels"]):
            x1, y1, x2, y2 = [round(float(value), 2) for value in box.tolist()]
            item = {
                "label": str(label),
                "score": round(float(score), 6),
                "bbox_xyxy": [x1, y1, x2, y2],
            }
            detections.append(item)
            draw.rectangle((x1, y1, x2, y2), outline=(30, 220, 80), width=3)
            draw.text((max(0, x1), max(0, y1 - 15)), f"{label} {float(score):.2f}", fill=(30, 220, 80))

        output_name = f"{path.stem}.jpg"
        canvas.save(args.output_dir / output_name, quality=95)
        records.append({
            "image": path.name,
            "detections": detections,
            "latency_ms": round(latency_ms, 3),
        })
        print(f"[{index}/{len(images)}] {path.name}: {len(detections)} detections, {latency_ms:.0f} ms", flush=True)

    latencies = [item["latency_ms"] for item in records]
    report = {
        "format": "pre_map_vln.grounding_dino_smoke.v1",
        "backend": "grounding_dino_huggingface",
        "model": args.model,
        "device": str(device),
        "prompts": prompts,
        "box_threshold": args.box_threshold,
        "text_threshold": args.text_threshold,
        "image_count": len(records),
        "latency_ms": {
            "mean": round(statistics.mean(latencies), 3),
            "median": round(statistics.median(latencies), 3),
            "max": round(max(latencies), 3),
        },
        "records": records,
    }
    (args.output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report["latency_ms"], ensure_ascii=False, indent=2))
    print(f"Output: {args.output_dir}")


if __name__ == "__main__":
    main()
