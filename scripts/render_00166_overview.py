#!/usr/bin/env python3
"""Render a compact visual audit of the finished 00166 ground-layer run."""

from __future__ import annotations

import csv
import glob
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def main() -> None:
    run = Path("outputs/stage1_00166/ground_v5")
    grid = np.load(run / "observed_ground.npy")
    meta = json.loads((run / "observed_ground.json").read_text(encoding="utf-8"))
    truth = np.load(run / "truth_metrics.npz")
    target = np.asarray(truth["main_floor_target"], dtype=bool)
    origin = np.asarray(meta["origin_xy_m"], dtype=float)
    resolution = float(meta["resolution_m"])

    height, width = grid.shape
    map_rgb = np.zeros((height, width, 3), dtype=np.uint8)
    map_rgb[grid < 0] = (105, 105, 105)       # unknown
    map_rgb[grid == 0] = (242, 242, 242)      # observed free
    map_rgb[grid == 100] = (20, 20, 20)       # occupied
    map_rgb[(grid != 0) & (grid != 100) & (grid >= 0)] = (180, 180, 180)

    # Green outline: Habitat Pathfinder main-floor target.
    interior = target.copy()
    edge = interior.copy()
    edge[:-1, :] &= interior[1:, :]
    edge[1:, :] &= interior[:-1, :]
    edge[:, :-1] &= interior[:, 1:]
    edge[:, 1:] &= interior[:, :-1]
    outline = target & ~edge
    map_rgb[outline] = (30, 180, 90)

    # Red: residual unknown cells inside the true main-floor target.
    map_rgb[target & (grid < 0)] = (220, 55, 45)

    trajectory = []
    with (run / "bag_export/trajectory.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            x = (float(row["x"]) - origin[0]) / resolution
            y = (float(row["y"]) - origin[1]) / resolution
            if 0 <= x < width and 0 <= y < height:
                trajectory.append((int(round(x)), int(round(y))))

    canvas = Image.new("RGB", (1600, 1040), ( thirty := 32, thirty, thirty))
    draw = ImageDraw.Draw(canvas)
    draw.text((28, 20), "HM3D 00166 stage-1 visual audit (ground layer)", fill=(255, 255, 255))
    draw.text(
        (28, 48),
        "white=free  black=occupied  gray=unknown  green=Habitat navmesh target  red=residual target unknown",
        fill=(220, 220, 220),
    )

    map_image = Image.fromarray(map_rgb, mode="RGB").resize((700, 760), Image.Resampling.NEAREST)
    if len(trajectory) > 1:
        scale_x = 700 / width
        scale_y = 760 / height
        points = [(x * scale_x, y * scale_y) for x, y in trajectory]
        ImageDraw.Draw(map_image).line(points, fill=(255, 145, 0), width=3)
        ImageDraw.Draw(map_image).ellipse(
            (points[0][0] - 5, points[0][1] - 5, points[0][0] + 5, points[0][1] + 5),
            fill=(40, 120, 255),
        )
        ImageDraw.Draw(map_image).ellipse(
            (points[-1][0] - 5, points[-1][1] - 5, points[-1][0] + 5, points[-1][1] + 5),
            fill=(255, 70, 30),
        )
    canvas.paste(map_image, (28, 100))
    draw.text((28, 875), "orange=executed sensor trajectory; blue=start; red=end", fill=(220, 220, 220))

    frame_paths = sorted(Path(run / "episode").glob("frame_*.npz"))
    if frame_paths:
        indices = np.linspace(0, len(frame_paths) - 1, min(6, len(frame_paths))).astype(int)
        for slot, index in enumerate(indices):
            path = frame_paths[int(index)]
            with np.load(path) as frame:
                rgb = np.asarray(frame["rgb"], dtype=np.uint8)
            image = Image.fromarray(rgb, mode="RGB").resize((270, 203), Image.Resampling.LANCZOS)
            x = 790 + (slot % 2) * 390
            y = 100 + (slot // 2) * 250
            canvas.paste(image, (x, y))
            draw.text((x, y + 208), path.stem, fill=(235, 235, 235))

    draw.text((790, 875), "RGB samples are perspective views, not official room labels.", fill=(255, 210, 100))
    output = run / "00166_stage1_overview.png"
    canvas.save(output)
    print(output)


if __name__ == "__main__":
    main()
