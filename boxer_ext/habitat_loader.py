"""Boxer loader for PRE_MAP_VLN Habitat episode NPZ files."""

import json
from pathlib import Path

import cv2
import numpy as np
import torch

from loaders.base_loader import BaseLoader
from utils.tw.obb import ObbTW
from utils.tw.pose import PoseTW


def quat_xyzw_to_matrix(q):
    x, y, z, w = np.asarray(q, dtype=np.float32)
    return np.asarray([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float32)


class HabitatLoader(BaseLoader):
    camera = "habitat_rgbd"
    device_name = "Habitat-Sim HM3D"

    def __init__(self, episode_dir, start_frame=0, skip_frames=1, max_frames=99999):
        self.root = Path(episode_dir)
        manifest_path = self.root / "manifest.json"
        if not manifest_path.exists():
            raise FileNotFoundError(f"Habitat manifest not found: {manifest_path}")
        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.frames = sorted(self.root.glob("frame_*.npz"))[start_frame::skip_frames][:max_frames]
        if not self.frames:
            raise RuntimeError(f"No frame_*.npz files in {self.root}")
        self.length, self.index, self.resize = len(self.frames), 0, None
        self._init_prefetch()

    def load(self, idx):
        with np.load(self.frames[idx]) as frame:
            rgb = np.asarray(frame["rgb"], dtype=np.uint8)
            depth = np.asarray(frame["depth_m"], dtype=np.float32)
            position = np.asarray(frame["position"], dtype=np.float32)
            orientation = np.asarray(frame["orientation_xyzw"], dtype=np.float32)
            time_ns = int(frame["time_ns"])
        h, w = rgb.shape[:2]
        fx = float(self.manifest.get("fx", 320.0))
        fy = float(self.manifest.get("fy", 320.0))
        cx = float(self.manifest.get("cx", w / 2.0))
        cy = float(self.manifest.get("cy", h / 2.0))
        if self.resize is not None:
            scale_x, scale_y = self.resize / w, self.resize / h
            rgb = cv2.resize(rgb, (self.resize, self.resize), interpolation=cv2.INTER_LINEAR)
            depth = cv2.resize(depth, (self.resize, self.resize), interpolation=cv2.INTER_NEAREST)
            w = h = self.resize
            fx, fy, cx, cy = fx * scale_x, fy * scale_y, cx * scale_x, cy * scale_y
        rotation = quat_xyzw_to_matrix(orientation)
        pose_data = torch.from_numpy(np.concatenate([rotation.reshape(-1), position]))
        return {
            "img0": self.img_to_tensor(rgb),
            "cam0": self.pinhole_from_K(w, h, fx, fy, cx, cy).float(),
            "T_world_rig0": PoseTW(pose_data.float()),
            "sdp_w": self.sdp_from_depth(depth, fx, fy, cx, cy, rotation, position),
            "time_ns0": time_ns,
            "bb2d0": torch.zeros(0, 4, dtype=torch.float32),
            "obbs": ObbTW(torch.zeros(0, 165)),
        }
