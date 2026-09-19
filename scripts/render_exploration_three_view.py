#!/usr/bin/env python3
"""Render a low-overhead three-view exploration preview from timeline snapshots."""

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
    parser = argparse.ArgumentParser()
    parser.add_argument("timeline", type=Path)
    parser.add_argument("rgb_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args()

    data = np.load(args.timeline, allow_pickle=False)
    all_traj = data["trajectory"].astype(np.float32)
    traj_times = data["trajectory_times"].astype(float)
    snapshot_times = data["snapshot_times"][:: max(1, args.stride)].astype(float)
    cloud_indices = list(range(0, 101, max(1, args.stride)))
    rgb_files = sorted(args.rgb_dir.glob("rgb_*.jpg"))
    if len(rgb_files) != len(cloud_indices):
        raise RuntimeError("RGB count %d does not match snapshot count %d" % (len(rgb_files), len(cloud_indices)))

    # Fit once against the completed map so the camera stays fixed without wasting space.
    final_cloud = data["cloud_100"].astype(np.float32)
    combined = np.vstack([final_cloud, all_traj])
    center = 0.5 * (combined.min(axis=0) + combined.max(axis=0))
    oblique_pitch = 0.72
    top_pitch = 0.0
    oblique_span = fitted_span(combined, 3.95, oblique_pitch, center)
    top_span = fitted_span(combined, 3.95, top_pitch, center)
    command = [
        "gst-launch-1.0", "-q", "fdsrc", "!",
        "videoparse", "format=rgb", "width=%d" % W, "height=%d" % H,
        "framerate=%d/1" % args.fps, "!", "videoconvert", "!",
        "x264enc", "speed-preset=ultrafast", "tune=zerolatency", "bitrate=5000", "key-int-max=%d" % (args.fps * 2), "!",
        "h264parse", "!", "mp4mux", "!", "filesink", "location=%s" % args.output,
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        for frame_no, (cloud_index, stamp, rgb_path) in enumerate(zip(cloud_indices, snapshot_times, rgb_files)):
            points = data["cloud_%03d" % cloud_index].astype(np.float32)
            trajectory = all_traj[traj_times <= stamp]
            frame = Image.new("RGB", (W, H), BG)
            first = fit_image(Image.open(rgb_path), LEFT_W, H - 72)
            frame.paste(first, (0, 56))
            oblique = render_view(points, trajectory, RIGHT_W, PANEL_H, 3.95, oblique_pitch, center, oblique_span, cloud_index)
            top = render_view(points, trajectory, RIGHT_W, PANEL_H, 3.95, top_pitch, center, top_span, cloud_index + 1000)
            frame.paste(oblique, (RIGHT_X, 56))
            frame.paste(top, (RIGHT_X, 568))
            draw = ImageDraw.Draw(frame)
            draw.text((20, 14), "第一视角", font=TITLE_FONT, fill=(235, 242, 250))
            draw.text((RIGHT_X + 18, 14), "斜俯视：三层探索结构", font=TITLE_FONT, fill=(235, 242, 250))
            draw.rectangle((RIGHT_X, 568, W, 608), fill=BG)
            draw.text((RIGHT_X + 18, 575), "正俯视：房间与轨迹", font=SMALL_FONT, fill=(235, 242, 250))
            progress = min(1.0, max(0.0, (stamp - float(data["bag_start"])) / (float(data["bag_end"]) - float(data["bag_start"]))))
            draw.rounded_rectangle((20, H - 34, W - 20, H - 18), radius=8, fill=(42, 58, 78))
            draw.rounded_rectangle((20, H - 34, 20 + int((W - 40) * progress), H - 18), radius=8, fill=(45, 190, 220))
            process.stdin.write(np.asarray(frame, dtype=np.uint8).tobytes())
            print("rendered %d/%d" % (frame_no + 1, len(cloud_indices)), flush=True)
    finally:
        if process.stdin:
            process.stdin.close()
        code = process.wait()
        if code:
            raise RuntimeError("GStreamer exited with code %d" % code)


if __name__ == "__main__":
    main()
