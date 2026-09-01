#!/usr/bin/env python3
"""Compose synchronized global, first-person, and third-person videos."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", required=True, type=Path)
    parser.add_argument("--right-top", required=True, type=Path)
    parser.add_argument("--right-bottom", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--left-width", type=int, default=1280)
    parser.add_argument("--right-width", type=int, default=640)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--divider-width", type=int, default=4)
    return parser.parse_args()


def info(cap: cv2.VideoCapture, path: Path) -> tuple[int, int, int, float]:
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {path}")
    values = (
        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
        float(cap.get(cv2.CAP_PROP_FPS)),
    )
    if min(values) <= 0:
        raise RuntimeError(f"invalid video metadata: {path}")
    return values


def aspect_fill(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    source_height, source_width = frame.shape[:2]
    scale = max(width / source_width, height / source_height)
    resized_width = int(round(source_width * scale))
    resized_height = int(round(source_height * scale))
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
    x0 = (resized_width - width) // 2
    y0 = (resized_height - height) // 2
    return resized[y0:y0 + height, x0:x0 + width]


def main() -> None:
    args = parse_args()
    if min(args.left_width, args.right_width, args.height) <= 0 or args.divider_width < 0:
        raise ValueError("panel dimensions must be positive and divider non-negative")
    right_height = (args.height - args.divider_width) // 2
    bottom_height = args.height - args.divider_width - right_height
    paths = (args.left, args.right_top, args.right_bottom)
    caps = [cv2.VideoCapture(str(path)) for path in paths]
    try:
        metadata = [info(cap, path) for cap, path in zip(caps, paths)]
        counts = [item[2] for item in metadata]
        rates = [item[3] for item in metadata]
        if len(set(counts)) != 1:
            raise ValueError(f"frame count mismatch: {counts}")
        if any(abs(rate - args.fps) > 1e-3 for rate in rates):
            raise ValueError(f"fps mismatch: inputs={rates}, output={args.fps}")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        writer = imageio.get_writer(
            args.output,
            fps=args.fps,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_log_level="error",
            output_params=["-movflags", "+faststart"],
        )
        try:
            for index in range(counts[0]):
                decoded = [cap.read() for cap in caps]
                if not all(ok for ok, _ in decoded):
                    raise RuntimeError(f"decode failed at frame {index}")
                left = aspect_fill(decoded[0][1], args.left_width, args.height)
                top = aspect_fill(decoded[1][1], args.right_width, right_height)
                bottom = aspect_fill(decoded[2][1], args.right_width, bottom_height)
                horizontal_divider = np.full(
                    (args.divider_width, args.right_width, 3), 18, dtype=np.uint8
                )
                right = np.concatenate([top, horizontal_divider, bottom], axis=0)
                vertical_divider = np.full(
                    (args.height, args.divider_width, 3), 18, dtype=np.uint8
                )
                composed = np.concatenate([left, vertical_divider, right], axis=1)
                writer.append_data(cv2.cvtColor(composed, cv2.COLOR_BGR2RGB))
                if index == 0 or (index + 1) % max(1, counts[0] // 10) == 0:
                    print(f"composed {index + 1}/{counts[0]}", flush=True)
        finally:
            writer.close()
    finally:
        for cap in caps:
            cap.release()
    print(f"video={args.output}")


if __name__ == "__main__":
    main()
