"""Shortest-path, rate-limited yaw helpers for simulated UAV execution."""

from __future__ import annotations

import math


def wrap_yaw(yaw: float) -> float:
    return math.atan2(math.sin(float(yaw)), math.cos(float(yaw)))


def yaw_delta(start: float, target: float) -> float:
    """Signed shortest angular displacement from start to target."""
    return wrap_yaw(float(target) - float(start))


def step_yaw(start: float, target: float, maximum_step_rad: float) -> float:
    maximum = max(1e-6, float(maximum_step_rad))
    delta = yaw_delta(start, target)
    return wrap_yaw(float(start) + max(-maximum, min(maximum, delta)))


def blend_yaw(start: float, target: float, fraction: float) -> float:
    ratio = max(0.0, min(1.0, float(fraction)))
    return wrap_yaw(float(start) + ratio * yaw_delta(start, target))


def rotation_steps(start: float, target: float, maximum_step_rad: float) -> list[float]:
    """Return rate-limited yaw samples excluding start and including target."""
    maximum = max(1e-6, float(maximum_step_rad))
    delta = yaw_delta(start, target)
    count = int(math.ceil(abs(delta) / maximum))
    if count == 0:
        return []
    return [wrap_yaw(float(start) + delta * index / count) for index in range(1, count + 1)]

