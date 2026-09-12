#!/usr/bin/env python3
"""Overlay compact closed-loop task state on a composed Stage-2 video."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np


LABELS = {
    "check_toilet_paper_l2_bathroom": "Toilet paper beside toilet",
    "find_backpack_under_table_l2": "Backpack below table",
    "find_backpack_under_bed_l2_bedroom": "Fallback: backpack below bed",
    "check_microwave_kitchen": "Kitchen microwave",
    "check_tv_l3": "Third-floor television",
}
ORDER = list(LABELS)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--execution", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--end-hold-s", type=float, default=2.0)
    args = parser.parse_args()

    execution = json.loads(args.execution.read_text(encoding="utf-8"))
    completion = {}
    scores = {}
    for item in execution.get("observations", []):
        if item.get("outcome") != "found":
            continue
        task_id = item["task_id"]
        completion[task_id] = int(item["frame_index"])
        detections = ((item.get("verification", {}).get("owlv2") or {}).get("detections") or [])
        if detections:
            scores[task_id] = float(detections[0]["score"])
    backpack_frame = completion["find_backpack_under_table_l2"]

    cap = cv2.VideoCapture(str(args.input))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {args.input}")
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        args.output, fps=fps, codec="libx264", quality=8, macro_block_size=1,
        ffmpeg_log_level="error", output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
    )
    last = None
    try:
        for index in range(frame_count):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f"decode failed at frame {index}")
            overlay = frame.copy()
            cv2.rectangle(overlay, (28, 92), (650, 350), (18, 22, 26), -1)
            cv2.addWeighted(overlay, 0.82, frame, 0.18, 0, frame)
            cv2.putText(frame, "CLOSED-LOOP TASK STATUS", (52, 128),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.82, (245, 245, 245), 2, cv2.LINE_AA)
            y = 168
            for task_id in ORDER:
                if task_id == "find_backpack_under_bed_l2_bedroom":
                    if index >= backpack_frame:
                        state, color = "SKIPPED", (0, 205, 255)
                    else:
                        state, color = "ARMED", (160, 180, 205)
                elif index >= completion.get(task_id, frame_count + 1):
                    state, color = "FOUND", (40, 225, 90)
                else:
                    next_pending = next(
                        (tid for tid in ORDER if tid != "find_backpack_under_bed_l2_bedroom"
                         and index < completion.get(tid, frame_count + 1)), None
                    )
                    state = "ACTIVE" if task_id == next_pending else "PENDING"
                    color = (70, 190, 255) if state == "ACTIVE" else (155, 165, 175)
                suffix = f" {scores[task_id]:.3f}" if state == "FOUND" and task_id in scores else ""
                cv2.circle(frame, (55, y - 7), 7, color, -1, cv2.LINE_AA)
                cv2.putText(frame, f"{LABELS[task_id]}  [{state}{suffix}]", (75, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.57, color, 2, cv2.LINE_AA)
                y += 37
            if backpack_frame <= index < backpack_frame + int(round(3.0 * fps)):
                cv2.rectangle(frame, (710, 94), (1330, 142), (15, 115, 45), -1)
                cv2.putText(frame, "Backpack found -> bed fallback SKIPPED", (730, 127),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
            writer.append_data(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            last = frame
        for _ in range(int(round(args.end_hold_s * fps))):
            writer.append_data(cv2.cvtColor(last, cv2.COLOR_BGR2RGB))
    finally:
        cap.release()
        writer.close()
    print(f"video={args.output} frames={frame_count + int(round(args.end_hold_s * fps))} fps={fps}")


if __name__ == "__main__":
    main()
