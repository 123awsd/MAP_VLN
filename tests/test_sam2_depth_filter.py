import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "boxer_ext"))

from sam2_depth_filter import filter_box_background_depth  # noqa: E402


def test_mask_removes_only_background_inside_detection_box():
    depth = np.ones((8, 10), dtype=np.float32)
    boxes = np.asarray([[2, 8, 1, 7]], dtype=np.float32)
    masks = np.zeros((1, 8, 10), dtype=bool)
    masks[0, 3:5, 4:6] = True

    filtered, stats = filter_box_background_depth(depth, boxes, masks, dilation_pixels=0)

    assert np.all(filtered[:, :2] == 1)
    assert np.all(filtered[3:5, 4:6] == 1)
    assert filtered[1, 2] == 0
    assert stats["valid_masks"] == 1
    assert stats["removed_background_depth"] == 32
    assert stats["box_depth_retention"] == 4 / 36


def test_empty_mask_is_fail_open():
    depth = np.ones((4, 4), dtype=np.float32)
    filtered, stats = filter_box_background_depth(
        depth,
        [[0, 4, 0, 4]],
        np.zeros((1, 4, 4), dtype=bool),
    )
    np.testing.assert_array_equal(filtered, depth)
    assert stats["valid_masks"] == 0
    assert stats["removed_background_depth"] == 0


def test_overlapping_foreground_is_preserved():
    depth = np.ones((6, 6), dtype=np.float32)
    boxes = [[0, 4, 0, 4], [2, 6, 2, 6]]
    masks = np.zeros((2, 6, 6), dtype=bool)
    masks[0, 1:3, 1:3] = True
    masks[1, 3:5, 3:5] = True
    filtered, _ = filter_box_background_depth(depth, boxes, masks, dilation_pixels=0)
    assert filtered[2, 2] == 1
    assert filtered[3, 3] == 1
    assert filtered[0, 0] == 0
    assert filtered[5, 0] == 1
