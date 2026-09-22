"""Geometry-only policy helpers for narrow-corridor diagnostics and tests.

The ROS1 planner has the authoritative C++ implementation of the corridor
intersection.  These small, dependency-free helpers intentionally mirror the
geometric policy so offline reports and regression tests can exercise the
failure modes without starting ROS.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

import numpy as np


class NarrowCorridorInfeasible(ValueError):
    """Raised when a centered tube cannot contain the corridor seed."""


def robust_local_tangent(points: Sequence[Sequence[float]], index: int,
                         minimum_span_m: float = 0.05) -> np.ndarray:
    """Estimate a horizontal direction while skipping repeated guide points."""
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] < 2 or not 0 <= index < len(values):
        raise ValueError("points/index are invalid")
    previous = next_index = index
    previous_span = next_span = 0.0
    while previous > 0 and previous_span < minimum_span_m:
        previous_span += float(np.linalg.norm(values[previous, :2] - values[previous - 1, :2]))
        previous -= 1
    while next_index + 1 < len(values) and next_span < minimum_span_m:
        next_span += float(np.linalg.norm(values[next_index + 1, :2] - values[next_index, :2]))
        next_index += 1
    tangent = values[next_index, :2] - values[previous, :2]
    norm = float(np.linalg.norm(tangent))
    if norm <= 1e-9:
        raise ValueError("guide has no local horizontal direction")
    return tangent / norm


def detect_two_sided_bottleneck(
    point: Sequence[float], tangent: Sequence[float], obstacles: Iterable[Sequence[float]],
    *, clearance_threshold_m: float = 0.40, max_width_m: float = 1.0,
    vertical_window_m: float = 0.35, longitudinal_window_m: float = 0.35,
) -> dict | None:
    """Return side clearances and midpoint for a genuine two-sided bottleneck.

    A single nearby wall is deliberately insufficient.  Distances are measured
    along the horizontal normal, with a small dead-band to avoid counting the
    path's own voxel or numerical noise as a wall.
    """
    p = np.asarray(point, dtype=float)
    t = np.asarray(tangent, dtype=float)[:2]
    t_norm = float(np.linalg.norm(t))
    if p.shape[0] < 3 or t_norm <= 1e-9:
        raise ValueError("point/tangent are invalid")
    t /= t_norm
    n = np.asarray([-t[1], t[0]])
    cloud = np.asarray(list(obstacles), dtype=float)
    if cloud.size == 0:
        return None
    cloud = cloud.reshape((-1, 3))
    delta = cloud - p[:3]
    mask = (
        (np.abs(delta[:, 2]) <= vertical_window_m)
        & (np.abs(delta[:, :2] @ t) <= longitudinal_window_m)
    )
    if not np.any(mask):
        return None
    lateral = delta[mask, :2] @ n
    positive = lateral[lateral > 0.05]
    negative = -lateral[lateral < -0.05]
    if not len(positive) or not len(negative):
        return None
    left = float(np.min(positive))
    right = float(np.min(negative))
    width = left + right
    nearest = float(np.min(np.linalg.norm(delta, axis=1)))
    if nearest > clearance_threshold_m or width > max_width_m:
        return None
    center = p.copy()
    center[:2] += n * (0.5 * (left - right))
    return dict(
        path_clearance_m=nearest,
        left_clearance_m=left,
        right_clearance_m=right,
        width_m=width,
        centerline=center,
        tangent=np.asarray([t[0], t[1], 0.0]),
        tube_normal=np.asarray([n[0], n[1], 0.0]),
    )


def centered_tube_half_width(seed_start: Sequence[float], seed_end: Sequence[float],
                             centerline: Sequence[float], normal: Sequence[float],
                             *, available_half_width_m: float,
                             minimum_half_width_m: float = 0.10,
                             maximum_half_width_m: float = 0.18,
                             margin_m: float = 0.01) -> float:
    """Choose a feasible half-width or raise instead of silently widening it."""
    start = np.asarray(seed_start, dtype=float)[:2]
    end = np.asarray(seed_end, dtype=float)[:2]
    center = np.asarray(centerline, dtype=float)[:2]
    n = np.asarray(normal, dtype=float)[:2]
    norm = float(np.linalg.norm(n))
    if norm <= 1e-9:
        raise NarrowCorridorInfeasible("tube normal is degenerate")
    n /= norm
    endpoint_span = max(abs(float((start - center) @ n)), abs(float((end - center) @ n)))
    half_width = min(maximum_half_width_m, max(available_half_width_m, endpoint_span + margin_m))
    if half_width < minimum_half_width_m or endpoint_span > half_width - 0.005:
        raise NarrowCorridorInfeasible(
            f"seed endpoint span {endpoint_span:.3f} m exceeds centered tube {half_width:.3f} m"
        )
    return float(half_width)


def unsigned_direction_error_deg(direction: Sequence[float], reference: Sequence[float]) -> float:
    """Smallest angle between two horizontal directions, ignoring orientation."""
    a = np.asarray(direction, dtype=float)[:2]
    b = np.asarray(reference, dtype=float)[:2]
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na <= 1e-9 or nb <= 1e-9:
        return math.nan
    cosine = float(np.clip(abs(np.dot(a / na, b / nb)), -1.0, 1.0))
    return math.degrees(math.acos(cosine))
