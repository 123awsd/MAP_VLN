#!/usr/bin/env python3
"""Compose two synchronized videos side by side without changing their timing."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", required=True, type=Path)
    parser.add_argument("--right", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--divider-width", type=int, default=4)
    parser.add_argument(
        "--right-scale",
        type=float,
        default=1.0,
        help="Scale the right 16:9 panel relative to the left panel.",
    )
    return parser.parse_args()


def video_info(capture: cv2.VideoCapture, path: Path) -> tuple[int, int, int, float]:
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {path}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if min(width, height, frames) <= 0 or fps <= 0:
        raise RuntimeError(f"invalid video metadata: {path}")
    return width, height, frames, fps


def main() -> None:
    args = parse_args()
    if args.divider_width < 0:
        raise ValueError("divider width cannot be negative")
    if not 0.1 <= args.right_scale <= 1.0:
        raise ValueError("right scale must be between 0.1 and 1.0")
    left = cv2.VideoCapture(str(args.left))
    right = cv2.VideoCapture(str(args.right))
    try:
        lw, lh, lf, lfps = video_info(left, args.left)
        rw, rh, rf, rfps = video_info(right, args.right)
        if lf != rf:
            raise ValueError(f"frame count mismatch: left={lf}, right={rf}")
        fps = float(args.fps if args.fps is not None else lfps)
        if abs(lfps - rfps) > 1e-3 or abs(fps - lfps) > 1e-3:
            raise ValueError(f"fps mismatch: left={lfps}, right={rfps}, output={fps}")

        target_height = min(lh, rh)
        left_width = int(round(lw * target_height / lh))
        right_height = int(round(target_height * args.right_scale))
        right_width = int(round(rw * right_height / rh))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        writer = imageio.get_writer(
            args.output,
            fps=fps,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_log_level="error",
            output_params=["-movflags", "+faststart"],
        )
        try:
            for index in range(lf):
                left_ok, left_frame = left.read()
                right_ok, right_frame = right.read()
                if not left_ok or not right_ok:
                    raise RuntimeError(f"decode failed at frame {index}")
                if (lw, lh) != (left_width, target_height):
                    left_frame = cv2.resize(
                        left_frame, (left_width, target_height), interpolation=cv2.INTER_AREA
                    )
                if (rw, rh) != (right_width, right_height):
                    right_frame = cv2.resize(
                        right_frame, (right_width, right_height), interpolation=cv2.INTER_AREA
                    )
                if right_height != target_height:
                    top = (target_height - right_height) // 2
                    bottom = target_height - right_height - top
                    right_frame = cv2.copyMakeBorder(
                        right_frame,
                        top,
                        bottom,
                        0,
                        0,
                        cv2.BORDER_CONSTANT,
                        value=(18, 18, 18),
                    )
                parts = [left_frame]
                if args.divider_width:
                    parts.append(np.full(
                        (target_height, args.divider_width, 3), 18, dtype=np.uint8
                    ))
                parts.append(right_frame)
                composed = np.concatenate(parts, axis=1)
                writer.append_data(cv2.cvtColor(composed, cv2.COLOR_BGR2RGB))
                if index == 0 or (index + 1) % max(1, lf // 10) == 0:
                    print(f"composed {index + 1}/{lf}", flush=True)
        finally:
            writer.close()
    finally:
        left.release()
        right.release()
    print(f"video={args.output}")


if __name__ == "__main__":
    main()
