"""Fast Open-Nav perception port backed by the project's persistent OWLv2 worker."""

from __future__ import annotations

import atexit
import os
from pathlib import Path
from typing import Any

import numpy as np

from stage2.open_vocab_detector import LocalOpenVocabularyDetector


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INDOOR_PROMPTS = [
    "bed", "bench", "cabinet", "chair", "clock", "counter", "couch", "curtain",
    "desk", "door", "dresser", "fireplace", "lamp", "mirror", "painting", "plant",
    "refrigerator", "shelf", "sink", "sofa", "stairs", "table", "television", "toilet",
    "towel", "vase", "window",
]


class OpenNavObservationBridge:
    """Expose Open-Nav's ``observe_view`` interface without SpatialBot/RAM.

    This is a controlled shared-perception port, not the upstream perception
    configuration. Object detection stays local and never calls Qwen.
    """

    def __init__(
        self,
        prompts: list[str] | None = None,
        *,
        detector: Any | None = None,
        runtime_dir: Path | None = None,
    ):
        self.prompts = prompts or list(DEFAULT_INDOOR_PROMPTS)
        self.runtime_dir = runtime_dir or ROOT / "runtime/baselines/opennav"
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.detector = detector or LocalOpenVocabularyDetector(
            self.prompts,
            threshold=0.20,
            cuda_visible_devices=os.getenv("PRE_MAP_VLN_OWLV2_CUDA_VISIBLE_DEVICES", "0"),
        )
        self._closed = False
        atexit.register(self.close)

    @staticmethod
    def _horizontal_region(center_x: float, width: int) -> str:
        ratio = center_x / max(1, width)
        if ratio < 0.38:
            return "left"
        if ratio > 0.62:
            return "right"
        return "center"

    def observe_view(
        self,
        logger: Any,
        current_step: int,
        direction_idx: str,
        direction_image: dict[str, Any],
    ) -> str:
        rgb = direction_image["rgb"]
        path = self.runtime_dir / f"step_{int(current_step):04d}_direction_{direction_idx}.png"
        if hasattr(rgb, "save"):
            rgb.save(path)
            width = int(rgb.size[0])
        else:
            from PIL import Image

            array = np.asarray(rgb, dtype=np.uint8)
            Image.fromarray(array).save(path)
            width = int(array.shape[1])
        result = self.detector.detect(path, prompts=self.prompts)
        descriptions = []
        labels = []
        for detection in sorted(
            result.get("detections", []), key=lambda item: float(item["score"]), reverse=True
        ):
            x1, _, x2, _ = detection["bbox_xyxy"]
            label = str(detection["label"]).strip().lower()
            if label in labels:
                continue
            region = self._horizontal_region((float(x1) + float(x2)) / 2.0, width)
            descriptions.append(f"{label} at image {region} (score {float(detection['score']):.2f})")
            labels.append(label)
        scene = "; ".join(descriptions) if descriptions else "no prompted landmark confidently detected"
        objects = ", ".join(dict.fromkeys(labels)) if labels else "none"
        observation = (
            f"Direction {direction_idx} Direction Viewpoint ID: {direction_idx} "
            f"in Step ID: {current_step} Elevation: Eye Level "
            f"Scene Description: {scene}. Scene Objects: {objects};"
        )
        if logger is not None:
            logger.info(observation)
        return observation

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        close = getattr(self.detector, "close", None)
        if close is not None:
            close()
