"""Mask-guided depth filtering used before BoxerNet 3-D lifting."""

from __future__ import annotations

from typing import Sequence

import numpy as np


def filter_box_background_depth(
    depth_m: np.ndarray,
    boxes_boxer: Sequence[Sequence[float]],
    masks: np.ndarray,
    dilation_pixels: int = 2,
) -> tuple[np.ndarray, dict[str, float | int]]:
    """Remove depth inside detector boxes when SAM 2 marks it as background.

    ``boxes_boxer`` use Boxer's ``x1,x2,y1,y2`` convention. Pixels outside all
    detector boxes remain untouched so BoxerNet retains global scene context.
    """
    import cv2

    depth = np.asarray(depth_m, dtype=np.float32)
    result = depth.copy()
    mask_array = np.asarray(masks, dtype=bool)
    boxes = np.asarray(boxes_boxer, dtype=np.float32).reshape(-1, 4)
    if mask_array.shape != (len(boxes), *depth.shape):
        raise ValueError(
            f"expected masks {(len(boxes), *depth.shape)}, got {mask_array.shape}"
        )

    box_union = np.zeros(depth.shape, dtype=bool)
    foreground_union = np.zeros(depth.shape, dtype=bool)
    valid_masks = 0
    kernel = None
    if dilation_pixels > 0:
        width = 2 * int(dilation_pixels) + 1
        kernel = np.ones((width, width), dtype=np.uint8)

    height, width = depth.shape
    for box, mask in zip(boxes, mask_array):
        x1, x2, y1, y2 = box.tolist()
        ix1, ix2 = max(0, int(np.floor(x1))), min(width, int(np.ceil(x2)))
        iy1, iy2 = max(0, int(np.floor(y1))), min(height, int(np.ceil(y2)))
        if ix2 <= ix1 or iy2 <= iy1:
            continue
        clipped = np.zeros_like(mask)
        clipped[iy1:iy2, ix1:ix2] = mask[iy1:iy2, ix1:ix2]
        if kernel is not None and clipped.any():
            clipped = cv2.dilate(clipped.astype(np.uint8), kernel, iterations=1).astype(bool)
            clipped[:iy1] = False
            clipped[iy2:] = False
            clipped[:, :ix1] = False
            clipped[:, ix2:] = False
        if not clipped.any():
            # A failed/empty mask must not erase a complete detection crop.
            continue
        box_union[iy1:iy2, ix1:ix2] = True
        foreground_union |= clipped
        valid_masks += 1

    removable = box_union & ~foreground_union & np.isfinite(depth) & (depth > 0)
    before = int((np.isfinite(depth) & (depth > 0)).sum())
    result[removable] = 0.0
    after = int((np.isfinite(result) & (result > 0)).sum())
    box_valid = int((box_union & np.isfinite(depth) & (depth > 0)).sum())
    retained_in_boxes = int((box_union & np.isfinite(result) & (result > 0)).sum())
    return result, {
        "detections": int(len(boxes)),
        "valid_masks": int(valid_masks),
        "valid_depth_before": before,
        "valid_depth_after": after,
        "removed_background_depth": before - after,
        "box_depth_pixels": box_valid,
        "box_depth_retention": 1.0 if box_valid == 0 else retained_in_boxes / box_valid,
    }

