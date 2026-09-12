"""SAM 2 box-prompted instance segmentation for the Boxer detector adapter."""

from __future__ import annotations

import os
from typing import Sequence

import numpy as np
import torch
from PIL import Image
from transformers import Sam2Model, Sam2Processor


class Sam2BoxSegmenter:
    """Turn detector boxes into one binary foreground mask per detection."""

    def __init__(
        self,
        device: str | torch.device = "cuda",
        model_name: str | None = None,
        mask_threshold: float = 0.0,
    ) -> None:
        requested = torch.device(device)
        self.device = torch.device("cpu") if requested.type == "mps" else requested
        self.model_name = model_name or os.environ.get(
            "PRE_MAP_VLN_SAM2_MODEL", "facebook/sam2.1-hiera-large"
        )
        self.mask_threshold = float(mask_threshold)
        print(f"==> Loading SAM 2: {self.model_name} on {self.device}", flush=True)
        self.processor = Sam2Processor.from_pretrained(self.model_name)
        self.model = Sam2Model.from_pretrained(self.model_name).to(self.device).eval()

    @torch.inference_mode()
    def segment(self, image: Image.Image | np.ndarray, boxes_xyxy: Sequence[Sequence[float]]) -> np.ndarray:
        """Return boolean masks shaped ``(N, H, W)`` in the input image size."""
        if isinstance(image, np.ndarray):
            image = Image.fromarray(np.asarray(image, dtype=np.uint8), mode="RGB")
        boxes = np.asarray(boxes_xyxy, dtype=np.float32).reshape(-1, 4)
        height, width = image.height, image.width
        if len(boxes) == 0:
            return np.zeros((0, height, width), dtype=bool)

        # SAM 2 expects a batch of images, each containing a list of object boxes.
        inputs = self.processor(
            images=image,
            input_boxes=[[box.tolist() for box in boxes]],
            return_tensors="pt",
        )
        model_inputs = {
            key: value.to(self.device) if hasattr(value, "to") else value
            for key, value in inputs.items()
            if key not in {"original_sizes", "reshaped_input_sizes"}
        }
        autocast_enabled = self.device.type == "cuda"
        with torch.autocast(
            device_type=self.device.type,
            dtype=torch.bfloat16,
            enabled=autocast_enabled,
        ):
            outputs = self.model(**model_inputs, multimask_output=True)

        masks = self.processor.post_process_masks(
            outputs.pred_masks.detach().cpu(),
            inputs["original_sizes"].detach().cpu(),
            mask_threshold=self.mask_threshold,
            binarize=True,
        )[0]
        scores = outputs.iou_scores.detach().cpu()[0]
        # Shapes are normally (objects, candidates, H, W) and
        # (objects, candidates). Keep the highest predicted-IoU mask.
        if masks.ndim == 3:
            masks = masks.unsqueeze(0)
        if scores.ndim == 1:
            scores = scores.unsqueeze(0)
        best = scores.argmax(dim=-1)
        selected = torch.stack([masks[i, best[i]] for i in range(len(boxes))])
        return selected.numpy().astype(bool, copy=False)

