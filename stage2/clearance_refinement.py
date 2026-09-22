"""Small, collision-checked XY-only ESDF refinement for 3-D paths."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from .voxel_map_3d import VoxelMap3D


def _sample_esdf_trilinear(voxel_map: VoxelMap3D, point: np.ndarray) -> float:
    """Trilinearly sample the ESDF, whose values are stored at voxel centers."""
    q = (point - voxel_map.origin) / voxel_map.resolution - 0.5
    nx, ny, nz = voxel_map.shape_xyz
    q = np.clip(q, [0.0, 0.0, 0.0], [nx - 1.0, ny - 1.0, nz - 1.0])
    x0, y0, z0 = np.floor(q).astype(int)
    tx, ty, tz = q - (x0, y0, z0)
    x0, y0, z0 = int(x0), int(y0), int(z0)
    x1, y1, z1 = min(x0 + 1, nx - 1), min(y0 + 1, ny - 1), min(z0 + 1, nz - 1)

    value = 0.0
    for dz, wz in ((0, 1.0 - tz), (1, tz)):
        for dy, wy in ((0, 1.0 - ty), (1, ty)):
            for dx, wx in ((0, 1.0 - tx), (1, tx)):
                value += float(voxel_map.esdf[z0 + dz * (z1 - z0),
                                             y0 + dy * (y1 - y0),
                                             x0 + dx * (x1 - x0)]) * wx * wy * wz
    return value


def _horizontal_gradient(
    voxel_map: VoxelMap3D, point: np.ndarray, sample_step_m: float,
) -> np.ndarray:
    gradient = np.zeros(2, dtype=float)
    for axis in range(2):
        offset = np.zeros(3, dtype=float)
        offset[axis] = sample_step_m
        gradient[axis] = (
            _sample_esdf_trilinear(voxel_map, point + offset)
            - _sample_esdf_trilinear(voxel_map, point - offset)
        ) / (2.0 * sample_step_m)
    return gradient


def _path_metrics(points: np.ndarray, voxel_map: VoxelMap3D) -> dict[str, float]:
    if len(points) == 0:
        return {"length_m": 0.0, "minimum_clearance_m": 0.0, "mean_clearance_m": 0.0}
    samples = [points[0]]
    max_step = max(0.025, voxel_map.resolution * 0.5)
    for start, end in zip(points, points[1:]):
        distance = float(np.linalg.norm(end - start))
        count = max(1, int(math.ceil(distance / max_step)))
        samples.extend(start + (end - start) * (i / count) for i in range(1, count + 1))
    clearances = [voxel_map.clearance(point) for point in samples]
    return {
        "length_m": float(sum(np.linalg.norm(b - a) for a, b in zip(points, points[1:]))),
        "minimum_clearance_m": float(min(clearances)),
        "mean_clearance_m": float(np.mean(clearances)),
    }


def refine_path_xy(
    path_xyz_m: Iterable[Iterable[float]],
    voxel_map: VoxelMap3D,
    *,
    target_clearance_m: float | None = None,
    max_lateral_shift_m: float = 0.20,
    max_step_m: float = 0.02,
    max_iterations: int = 20,
    endpoint_taper_m: float = 0.50,
) -> tuple[list[list[float]], dict[str, object]]:
    """Nudge interior path samples toward larger XY ESDF while preserving safety.

    Endpoints and every sample's z are fixed. A proposed iteration is accepted
    only when every resulting point and every connecting segment remains in the
    same inflated-free map used by A*. The shift is bounded relative to the
    original path, and a taper protects the start and observation approach.
    """
    original = np.asarray(list(path_xyz_m), dtype=float)
    if original.ndim != 2 or original.shape[1] != 3 or len(original) == 0:
        raise ValueError("path must be a non-empty Nx3 array")
    if not np.isfinite(original).all():
        raise ValueError("path contains non-finite coordinates")
    target = (voxel_map.profile.preferred_esdf_distance_m
              if target_clearance_m is None else float(target_clearance_m))
    if target < voxel_map.profile.minimum_esdf_distance_m:
        raise ValueError("target clearance cannot be below the hard ESDF threshold")
    if max_lateral_shift_m < 0.0 or max_step_m <= 0.0 or max_iterations < 0:
        raise ValueError("invalid refinement limits")
    if not all(voxel_map.line_is_valid(a, b) for a, b in zip(original, original[1:])):
        raise ValueError("input path is not collision-free in the inflated voxel map")

    before = _path_metrics(original, voxel_map)
    points = original.copy()
    if len(points) < 3 or max_iterations == 0 or max_lateral_shift_m == 0.0:
        return points.tolist(), {
            "algorithm": "esdf_xy_gradient_v1", "target_clearance_m": target,
            "iterations": 0, "max_lateral_shift_m": 0.0,
            "before": before, "after": before, "accepted": False,
        }

    cumulative = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(original, axis=0), axis=1))))
    total_length = float(cumulative[-1])
    distance_to_end = total_length - cumulative
    sample_step = max(0.025, voxel_map.resolution * 0.5)
    accepted_iterations = 0
    for _ in range(max_iterations):
        proposed_delta = np.zeros_like(points)
        for index in range(1, len(points) - 1):
            clearance = _sample_esdf_trilinear(voxel_map, points[index])
            deficit = target - clearance
            if deficit <= 1e-3:
                continue
            gradient = _horizontal_gradient(voxel_map, points[index], sample_step)
            norm = float(np.linalg.norm(gradient))
            if norm <= 1e-5:
                continue
            taper = 1.0
            if endpoint_taper_m > 0.0:
                taper = min(1.0, cumulative[index] / endpoint_taper_m,
                            distance_to_end[index] / endpoint_taper_m)
            if taper <= 0.0:
                continue
            # A bounded ascent step: alpha is reduced as clearance approaches
            # the preferred target, avoiding large jumps in already-open space.
            magnitude = min(max_step_m, max_step_m * min(1.0, deficit / max(target, 1e-6)))
            proposed_delta[index, :2] = gradient * (magnitude * taper / norm)

        if float(np.max(np.linalg.norm(proposed_delta[:, :2], axis=1))) <= 1e-6:
            break

        accepted = False
        scale = 1.0
        for _backtrack in range(7):
            candidate = points + proposed_delta * scale
            candidate[:, 2] = original[:, 2]
            lateral = np.linalg.norm(candidate[:, :2] - original[:, :2], axis=1)
            if (float(np.max(lateral)) <= max_lateral_shift_m + 1e-9
                    and all(voxel_map.is_state_valid(point) for point in candidate)
                    and all(voxel_map.line_is_valid(a, b) for a, b in zip(candidate, candidate[1:]))):
                points = candidate
                accepted = True
                accepted_iterations += 1
                break
            scale *= 0.5
        if not accepted:
            break

    points[0] = original[0]
    points[-1] = original[-1]
    points[:, 2] = original[:, 2]
    if not all(voxel_map.line_is_valid(a, b) for a, b in zip(points, points[1:])):
        raise RuntimeError("refined path failed final continuous collision recheck")
    after = _path_metrics(points, voxel_map)
    maximum_shift = float(np.max(np.linalg.norm(points[:, :2] - original[:, :2], axis=1)))
    return points.tolist(), {
        "algorithm": "esdf_xy_gradient_v1",
        "target_clearance_m": target,
        "iterations": accepted_iterations,
        "max_lateral_shift_m": maximum_shift,
        "configured_max_lateral_shift_m": float(max_lateral_shift_m),
        "z_unchanged": bool(np.array_equal(points[:, 2], original[:, 2])),
        "before": before,
        "after": after,
        "accepted": maximum_shift > 1e-6,
    }
