#!/usr/bin/env python3
"""Compose the demo layout used by demo1: third-person main, two right views."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--third", required=True, type=Path)
    parser.add_argument("--first", required=True, type=Path)
    parser.add_argument("--global-view", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--hold-final-s", type=float, default=0.0)
    return parser.parse_args()


def fit(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_LANCZOS4)


def main() -> None:
    args = parse_args()
    paths = (args.third, args.first, args.global_view)
    caps = [cv2.VideoCapture(str(path)) for path in paths]
    if not all(cap.isOpened() for cap in caps):
        raise RuntimeError(f"cannot open one of: {paths}")
    try:
        metadata = [
            (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
             int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
             int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
             float(cap.get(cv2.CAP_PROP_FPS)))
            for cap in caps
        ]
        counts = [item[2] for item in metadata]
        if len(set(counts)) != 1:
            raise ValueError(f"frame count mismatch: {metadata}")
        if any(abs(item[3] - args.fps) > 1e-3 for item in metadata):
            raise ValueError(f"fps mismatch: {metadata}, output={args.fps}")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        writer = imageio.get_writer(
            args.output,
            fps=args.fps,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_log_level="error",
            output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
        )
        last = None
        try:
            for index in range(counts[0]):
                decoded = [cap.read() for cap in caps]
                if not all(ok for ok, _ in decoded):
                    raise RuntimeError(f"decode failed at frame {index}")
                canvas = np.full((1080, 1920, 3), 255, dtype=np.uint8)
                canvas[180:900, 0:1280] = fit(decoded[0][1], 1280, 720)
                canvas[60:540, 1280:1920] = fit(decoded[1][1], 640, 480)
                canvas[600:960, 1280:1920] = fit(decoded[2][1], 640, 360)
                writer.append_data(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
                last = canvas
                if index == 0 or (index + 1) % max(1, counts[0] // 10) == 0:
                    print(f"composed {index + 1}/{counts[0]}", flush=True)
            if last is not None and args.hold_final_s > 0:
                for _ in range(int(round(args.hold_final_s * args.fps))):
                    writer.append_data(cv2.cvtColor(last, cv2.COLOR_BGR2RGB))
        finally:
            writer.close()
    finally:
        for cap in caps:
            cap.release()
    print(f"video={args.output}")


if __name__ == "__main__":
    main()
