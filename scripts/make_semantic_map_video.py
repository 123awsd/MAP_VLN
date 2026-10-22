#!/usr/bin/env python3
"""One-command build for the editable Blender scene, previews, and MP4."""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument("--config", type=Path, default=ROOT / "config/semantic_map_animation_9s.json")
p.add_argument("--preview-only", action="store_true")
p.add_argument("--encode-only", action="store_true")
args = p.parse_args()
config = args.config.resolve()
cfg = json.loads(config.read_text())
out = Path(cfg["output_dir"])
if not out.is_absolute():
    out = ROOT / out
out.mkdir(parents=True, exist_ok=True)
script = ROOT / "scripts/render_semantic_map_animation.py"
blender = shutil.which("blender")
if blender is None:
    raise RuntimeError("Blender executable was not found")
if not args.encode_only:
    for mode in ["preview"] if args.preview_only else ["preview", "video"]:
        subprocess.run([blender, "-b", "-t", "8", "--python", str(script), "--", "--config", str(config), "--mode", mode], check=True)
if not args.preview_only:
    frame_count = int(cfg["fps"] * cfg["seconds"])
    frames = sorted((out / "frames").glob("frame_*.png"))
    if len(frames) != frame_count:
        raise RuntimeError(f"Expected {frame_count} frames, found {len(frames)}")
    mp4 = out / "semantic_map_9s.mp4"
    subprocess.run([
        imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-v", "error", "-framerate", str(cfg["fps"]),
        "-i", str(out / "frames" / "frame_%04d.png"), "-frames:v", str(frame_count),
        "-c:v", "libx264", "-preset", "medium", "-crf", "19", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(mp4),
    ], check=True)
    print(mp4)
