#!/usr/bin/env python3
"""Persistent local OWLv2 worker using a JSON-lines protocol on stdio."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party/boxer"))

from owl.owl_wrapper import OwlWrapper  # noqa: E402


READY_PREFIX = "PRE_MAP_VLN_OWLV2_READY "
RESULT_PREFIX = "PRE_MAP_VLN_OWLV2_RESULT "


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prompts", nargs="+", required=True)
    parser.add_argument("--threshold", type=float, default=0.08)
    parser.add_argument("--precision", choices=("float32", "bfloat16"), default="bfloat16")
    parser.add_argument("--device", default="cuda")
    return parser.parse_args()


def emit(prefix: str, payload: dict) -> None:
    print(prefix + json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)


def main() -> int:
    args = parse_args()
    started = time.perf_counter()
    detector = OwlWrapper(
        device=args.device,
        text_prompts=args.prompts,
        min_confidence=args.threshold,
        precision=args.precision,
        warmup=True,
    )
    prompts = list(args.prompts)
    emit(READY_PREFIX, {
        "device": args.device,
        "prompts": prompts,
        "threshold": args.threshold,
        "startup_s": round(time.perf_counter() - started, 4),
    })

    for line in sys.stdin:
        try:
            request = json.loads(line)
            if request.get("command") == "stop":
                return 0
            request_prompts = request.get("prompts") or prompts
            if request_prompts != prompts:
                detector.set_text_prompts(request_prompts)
                prompts = list(request_prompts)
            image_path = Path(request["image_path"])
            image = np.asarray(Image.open(image_path).convert("RGB"))
            image_tensor = torch.from_numpy(image.copy()).permute(2, 0, 1).float()[None]
            if args.device == "cuda":
                torch.cuda.synchronize()
            infer_started = time.perf_counter()
            boxes, scores, labels, _ = detector.forward(image_tensor)
            if args.device == "cuda":
                torch.cuda.synchronize()
            latency_ms = (time.perf_counter() - infer_started) * 1000.0
            detections = []
            for box, score, label_index in zip(boxes.tolist(), scores.tolist(), labels.tolist()):
                x1, x2, y1, y2 = box
                detections.append({
                    "label": prompts[int(label_index)],
                    "score": round(float(score), 6),
                    "bbox_xyxy": [round(float(x1), 2), round(float(y1), 2), round(float(x2), 2), round(float(y2), 2)],
                })
            detections.sort(key=lambda item: item["score"], reverse=True)
            emit(RESULT_PREFIX, {
                "id": request.get("id"),
                "image_path": str(image_path),
                "image_size": [int(image.shape[1]), int(image.shape[0])],
                "latency_ms": round(latency_ms, 3),
                "detections": detections,
            })
        except Exception as error:
            emit(RESULT_PREFIX, {
                "id": request.get("id") if "request" in locals() else None,
                "error": f"{type(error).__name__}: {error}",
            })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
