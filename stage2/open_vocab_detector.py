"""Client and geometry helpers for the persistent local OWLv2 detector."""

from __future__ import annotations

import json
import math
import selectors
import subprocess
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
READY_PREFIX = "PRE_MAP_VLN_OWLV2_READY "
RESULT_PREFIX = "PRE_MAP_VLN_OWLV2_RESULT "


class LocalOpenVocabularyDetector:
    """Keep OWLv2 resident in its isolated Boxer environment."""

    def __init__(
        self,
        prompts: list[str],
        threshold: float = 0.08,
        timeout_s: float = 180.0,
        python_path: Path | None = None,
    ):
        if not prompts:
            raise ValueError("at least one prompt is required")
        self.prompts = list(dict.fromkeys(item.strip().lower() for item in prompts if item.strip()))
        self.timeout_s = timeout_s
        executable = python_path or ROOT / ".envs/boxer/bin/python"
        command = [
            str(executable), "-u", str(ROOT / "boxer_ext/open_vocab_worker.py"),
            "--threshold", str(threshold), "--prompts", *self.prompts,
        ]
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.ready = self._read_prefixed(READY_PREFIX, timeout_s)

    def _read_prefixed(self, prefix: str, timeout_s: float) -> dict[str, Any]:
        if self.process.stdout is None:
            raise RuntimeError("OWLv2 worker stdout is unavailable")
        selector = selectors.DefaultSelector()
        selector.register(self.process.stdout, selectors.EVENT_READ)
        deadline = time.monotonic() + timeout_s
        diagnostics: list[str] = []
        try:
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    remainder = self.process.stdout.read()
                    raise RuntimeError("OWLv2 worker exited: " + " | ".join((diagnostics + [remainder])[-8:]))
                events = selector.select(timeout=min(1.0, max(0.0, deadline - time.monotonic())))
                if not events:
                    continue
                line = self.process.stdout.readline().rstrip("\n")
                if line.startswith(prefix):
                    return json.loads(line[len(prefix):])
                if line:
                    diagnostics.append(line)
                    diagnostics = diagnostics[-8:]
        finally:
            selector.close()
        raise TimeoutError(f"OWLv2 worker timed out; recent output: {' | '.join(diagnostics)}")

    def detect(self, image_path: Path, prompts: list[str] | None = None) -> dict[str, Any]:
        if self.process.stdin is None:
            raise RuntimeError("OWLv2 worker stdin is unavailable")
        request_id = uuid.uuid4().hex
        request: dict[str, Any] = {"id": request_id, "image_path": str(Path(image_path).resolve())}
        if prompts is not None:
            request["prompts"] = prompts
        self.process.stdin.write(json.dumps(request, separators=(",", ":")) + "\n")
        self.process.stdin.flush()
        result = self._read_prefixed(RESULT_PREFIX, self.timeout_s)
        if result.get("id") != request_id:
            raise RuntimeError("OWLv2 worker returned a mismatched request id")
        if result.get("error"):
            raise RuntimeError(result["error"])
        return result

    def close(self) -> None:
        if self.process.poll() is not None:
            return
        if self.process.stdin is not None:
            try:
                self.process.stdin.write('{"command":"stop"}\n')
                self.process.stdin.flush()
            except BrokenPipeError:
                pass
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=5)

    def __enter__(self) -> "LocalOpenVocabularyDetector":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def target_found(result: dict[str, Any], target: str, aliases: dict[str, list[str]] | None = None) -> bool:
    accepted = {target.strip().lower()}
    if aliases:
        accepted.update(item.strip().lower() for item in aliases.get(target.strip().lower(), []))
    return any(item["label"].strip().lower() in accepted for item in result.get("detections", []))


def project_detection_to_world(
    detection: dict[str, Any],
    depth: np.ndarray,
    pose_xyzyaw: list[float],
    hfov_degrees: float = 90.0,
) -> dict[str, Any] | None:
    """Lift an aligned RGB detection with robust median depth into Falcon coordinates."""
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = detection["bbox_xyxy"]
    ix1, iy1 = max(0, int(x1)), max(0, int(y1))
    ix2, iy2 = min(width, int(math.ceil(x2))), min(height, int(math.ceil(y2)))
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    crop = np.asarray(depth[iy1:iy2, ix1:ix2], dtype=np.float32)
    valid = crop[np.isfinite(crop) & (crop > 0.05)]
    if valid.size < 8:
        return None
    distance = float(np.median(valid))
    focal = width / (2.0 * math.tan(math.radians(hfov_degrees) / 2.0))
    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    right = (u - width / 2.0) * distance / focal
    up = -(v - height / 2.0) * distance / focal
    forward = distance
    px, py, pz, yaw = map(float, pose_xyzyaw)
    left = -right
    world_x = px + math.cos(yaw) * forward - math.sin(yaw) * left
    world_y = py + math.sin(yaw) * forward + math.cos(yaw) * left
    box_width = max(0.1, (x2 - x1) * distance / focal)
    box_height = max(0.1, (y2 - y1) * distance / focal)
    return {
        "label": detection["label"],
        "score": detection["score"],
        "center": [round(world_x, 4), round(world_y, 4), round(pz + up, 4)],
        "size": [round(max(0.2, distance * 0.15), 4), round(box_width, 4), round(box_height, 4)],
        "depth_m": round(distance, 4),
    }


def associate_projection(
    projected: dict[str, Any],
    objects: list[dict[str, Any]],
    aliases: dict[str, set[str]] | None = None,
    maximum_distance_m: float = 1.0,
) -> dict[str, Any]:
    """Associate a lifted detection with the nearest same-class pre-map object."""
    label = projected["label"].strip().lower()
    accepted = {label}
    if aliases:
        accepted.update(aliases.get(label, set()))
    candidates = []
    for obj in objects:
        object_label = str(obj.get("label", "")).strip().lower()
        if object_label not in accepted and not any(item in object_label for item in accepted):
            continue
        distance = math.dist(projected["center"], obj["center_xyz_m"])
        candidates.append((distance, obj["id"]))
    result = dict(projected)
    if not candidates:
        result.update({"associated_object_id": None, "association_distance_m": None, "confirmed": False})
        return result
    distance, object_id = min(candidates)
    result.update({
        "associated_object_id": object_id,
        "association_distance_m": round(distance, 4),
        "confirmed": distance <= maximum_distance_m,
    })
    return result
