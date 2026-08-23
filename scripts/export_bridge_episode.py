#!/usr/bin/env python3
"""Export the latest atomic bridge observation as a one-frame Boxer episode."""

import json
import time
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "runtime/bridge"
OUTPUT = ROOT / "data/episodes/hm3d_boxer_smoke"


def main():
    state = json.loads((BRIDGE / "state.json").read_text(encoding="utf-8"))
    h, w = int(state["height"]), int(state["width"])
    rgb = np.fromfile(BRIDGE / "rgb_u8.raw", dtype=np.uint8).reshape(h, w, 3)
    depth = np.fromfile(BRIDGE / "depth_u16.raw", dtype="<u2").reshape(h, w).astype(np.float32) / 1000.0
    OUTPUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        OUTPUT / "frame_000000.npz", rgb=rgb, depth_m=depth,
        position=np.asarray(state["position"], dtype=np.float32),
        orientation_xyzw=np.asarray(state["orientation_xyzw"], dtype=np.float32),
        time_ns=np.asarray(time.time_ns(), dtype=np.int64),
    )
    (OUTPUT / "manifest.json").write_text(json.dumps({
        "format": "pre_map_vln.habitat_episode.v1", "scene": "00861-GLAQ4DNUx5U",
        "width": w, "height": h, "fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
        "coordinate_frame": "falcon_world_z_up_camera_optical",
    }, indent=2), encoding="utf-8")
    print(OUTPUT)


if __name__ == "__main__":
    main()
