"""Remove masked background only within valid object proposal footprints.

Boxes follow Boxer's x_min,x_max,y_min,y_max convention. Empty masks leave
their proposal unfiltered. Overlapping foreground masks protect each other.
Depth outside proposals is preserved for the model's scene context.
"""
import cv2
import numpy as np


def filter_box_background_depth(depth, boxes, masks, dilation_pixels=2):
    if dilation_pixels < 0:
        raise ValueError('dilation_pixels must be nonnegative')
    depth = np.asarray(depth)
    if depth.ndim != 2:
        raise ValueError('depth must be H x W')
    boxes = boxes.detach().cpu().numpy() if hasattr(boxes, 'detach') else np.asarray(boxes)
    masks = masks.detach().cpu().numpy() if hasattr(masks, 'detach') else np.asarray(masks)
    if len(boxes) != len(masks):
        raise ValueError('one mask is required per box')
    if len(boxes) and masks.shape != (len(boxes), *depth.shape):
        raise ValueError('masks must be N x H x W at depth resolution')
    footprint = np.zeros(depth.shape, bool)
    foreground = np.zeros(depth.shape, bool)
    valid_masks = 0
    h, w = depth.shape
    for box, mask in zip(boxes, masks):
        if len(box) != 4 or not np.isfinite(box).all():
            raise ValueError('invalid box coordinates')
        x1, x2, y1, y2 = box
        x1, y1 = max(0, int(np.floor(x1))), max(0, int(np.floor(y1)))
        x2, y2 = min(w, int(np.ceil(x2))), min(h, int(np.ceil(y2)))
        if x2 <= x1 or y2 <= y1:
            continue
        area = np.zeros(depth.shape, bool)
        area[y1:y2, x1:x2] = True
        footprint |= area
        mask = np.asarray(mask, bool)
        if not np.any(mask & area):
            foreground |= area
            continue
        valid_masks += 1
        if dilation_pixels:
            size = 2*dilation_pixels+1
            mask = cv2.dilate(mask.astype(np.uint8), np.ones((size, size), np.uint8)).astype(bool)
        foreground |= mask & area
    valid = np.isfinite(depth) & (depth > 0)
    remove = footprint & ~foreground & valid
    result = depth.copy()
    result[remove] = 0
    total = int((valid & footprint).sum())
    return result, {
        'detections': len(boxes), 'valid_masks': valid_masks,
        'valid_depth_before': int(valid.sum()),
        'valid_depth_after': int((valid & ~remove).sum()),
        'box_depth_pixels': total, 'removed_background_depth': int(remove.sum()),
        'box_depth_retention': float(1-remove.sum()/total) if total else 1.,
    }
