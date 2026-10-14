#!/usr/bin/env python3
"""Render the archived original exploration video; shared projection helpers."""

import argparse
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


W, H = 1920, 1080
LEFT_W = 704
RIGHT_X = 720
RIGHT_W = W - RIGHT_X
PANEL_H = 512
BG = (12, 22, 38)
PANEL_BG = (248, 250, 252)


def font(size):
    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for path in candidates:
        if Path(path).is_file():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


TITLE_FONT = font(26)
SMALL_FONT = font(20)


def fit_image(image, width, height):
    image = image.convert("RGB")
    scale = min(width / image.width, height / image.height)
    resized = image.resize((max(1, round(image.width * scale)), max(1, round(image.height * scale))), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (width, height), (0, 0, 0))
    canvas.paste(resized, ((width - resized.width) // 2, (height - resized.height) // 2))
    return canvas


def rotation(yaw, pitch):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]], dtype=np.float32)
    return rx @ rz


def colors_for(points, alpha_context=False):
    z = points[:, 2]
    ratio = np.clip((z - (-0.5)) / 10.8, 0, 1)
    red = np.interp(ratio, (0, 0.55, 1), (0, 35, 235))
    green = np.interp(ratio, (0, 0.55, 1), (210, 70, 0))
    blue = np.interp(ratio, (0, 0.55, 1), (230, 210, 210))
    rgb = np.column_stack([red, green, blue]).astype(np.uint8)
    if alpha_context:
        rgb = (0.42 * rgb + 0.58 * np.array([210, 220, 230])).astype(np.uint8)
    return rgb


def project(points, width, height, yaw, pitch, center, span, margin=26):
    if not len(points):
        return np.empty((0, 2), dtype=np.int32), np.empty(0)
    view = (points - center) @ rotation(yaw, pitch).T
    scale = min((width - 2 * margin) / span[0], (height - 2 * margin) / span[1])
    x = width / 2 + view[:, 0] * scale
    y = height / 2 - view[:, 1] * scale
    return np.column_stack([x, y]).astype(np.int32), view[:, 2]


def fitted_span(points, yaw, pitch, center, padding=1.12):
    view = (points - center) @ rotation(yaw, pitch).T
    ranges = np.maximum(1.0, view[:, :2].max(axis=0) - view[:, :2].min(axis=0))
    return tuple((ranges * padding).tolist())


def render_view(points, trajectory, width, height, yaw, pitch, center, span, seed):
    image = Image.new("RGB", (width, height), PANEL_BG)
    draw = ImageDraw.Draw(image)
    if len(points):
        max_points = 28000
        if len(points) > max_points:
            rng = np.random.default_rng(seed)
            indices = rng.choice(len(points), max_points, replace=False)
            pts = points[indices]
        else:
            pts = points
        pixels, depth = project(pts, width, height, yaw, pitch, center, span)
        order = np.argsort(depth)
        rgb = colors_for(pts)
        for idx in order:
            x, y = pixels[idx]
            if 0 <= x < width and 0 <= y < height:
                color = tuple(int(v) for v in rgb[idx])
                draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=color)
    if len(trajectory) > 1:
        path, _ = project(trajectory, width, height, yaw, pitch, center, span)
        path = [(int(x), int(y)) for x, y in path if 0 <= x < width and 0 <= y < height]
        if len(path) > 1:
            draw.line(path, fill=(18, 55, 170), width=3, joint="curve")
            x, y = path[-1]
            draw.ellipse((x - 6, y - 6, x + 6, y + 6), fill=(245, 95, 35), outline=(255, 255, 255), width=2)
    return image


def main():
    """Default to the archived Oct13 original layout; retain projection helpers."""
    import shutil
    parser = argparse.ArgumentParser(description="前天原版：无字、俯视主图、右上三层点云、右下规划yaw第一视角")
    parser.add_argument("--trial-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cache-dir", type=Path, help="默认 trial-dir/video_cache")
    parser.add_argument("--assets-dir", type=Path, help="默认 trial-dir/original_style_video；含本次几何及离线yaw素材")
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    workspace = project.parent
    cache = (args.cache_dir or args.trial_dir/"video_cache").resolve()
    assets = (args.assets_dir or args.trial_dir/"original_style_video").resolve()
    archive = project/"visualization/current_video_snapshot/探索剪辑素材_无字30帧"
    import json
    config = assets/"生成配置.json"
    settings = json.loads((config if config.exists() else archive/"生成配置.json").read_text())
    required = [cache/"increments.npz", cache/"timeline.npz", cache/"camera_pose.npy", assets/"geometry.npz"]
    if settings.get("yaw_matched_camera", False):
        required += [assets/"yaw_matched_camera_pose.npy", assets/"yaw_matched_rgb"]
    missing = [str(p) for p in required if not p.exists()]
    if missing:
        parser.error("原版素材尚未准备完成：" + ", ".join(missing))
    config = assets/"生成配置.json"
    if not config.exists():
        shutil.copyfile(archive/"生成配置.json", config)
    source = (archive/"render.py").read_text()
    source = source.replace("ROOT=Path(__file__).resolve().parents[2]", "ROOT=Path(" + repr(str(workspace)) + ")")
    source = source.replace("ROOT/'PRE_MAP_VLN/outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed'", "Path(" + repr(str(cache)) + ")")
    source = source.replace("np.load(Path(__file__).resolve().parent/'geometry.npz')", "np.load(Path(" + repr(str(assets/"geometry.npz")) + "))")
    source = source.replace("OUT=ROOT/'视频制作/探索剪辑素材_无字30帧'", "OUT=Path(" + repr(str(assets)) + ")")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    source = source.replace("'04_合并版_无字_30fps.mp4'", repr(str(output)))
    generated = assets/"render_default_original.py"
    generated.write_text(source)
    subprocess.run([__import__("sys").executable, str(generated)], check=True)


if __name__ == "__main__":
    main()
