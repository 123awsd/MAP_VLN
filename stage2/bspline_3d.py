"""Collision-checked B-spline trajectories for the Habitat simulation backend."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import splprep, splev

from .voxel_map_3d import VoxelMap3D


@dataclass(frozen=True)
class BsplineSettings:
    sample_step_m: float = 0.05
    smoothing_candidates: tuple[float, ...] = (0.03, 0.01, 0.0)
    conditional_prefix_length_m: float = 0.70

    @classmethod
    def load(cls, path: Path) -> "BsplineSettings":
        value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        config = value.get("bspline", {})
        result = cls(
            sample_step_m=float(config.get("sample_step_m", 0.05)),
            smoothing_candidates=tuple(
                float(item) for item in config.get("smoothing_candidates", (0.03, 0.01, 0.0))
            ),
            conditional_prefix_length_m=float(
                config.get("conditional_prefix_length_m", 0.70)
            ),
        )
        if result.sample_step_m <= 0 or result.conditional_prefix_length_m <= 0:
            raise ValueError("B-spline sample step and prefix length must be positive")
        if not result.smoothing_candidates or min(result.smoothing_candidates) < 0:
            raise ValueError("B-spline smoothing candidates must be non-negative")
        return result


@dataclass(frozen=True)
class BsplinePath3D:
    points_xyz_m: list[list[float]]
    mode: str
    degree: int
    smoothing_m: float
    piece_count: int
    minimum_clearance_m: float


def anchor_astar_path(
    astar_points_xyz_m: list[list[float]],
    start_xyz_m: list[float],
    goal_xyz_m: list[float],
    voxel_map: VoxelMap3D,
) -> list[list[float]]:
    """Connect snapped A* cell centers to the actual executable endpoints."""
    points = _deduplicate(astar_points_xyz_m).tolist()
    for label, endpoint in (("start", start_xyz_m), ("goal", goal_xyz_m)):
        if not voxel_map.is_state_valid(endpoint):
            raise RuntimeError(f"actual B-spline {label} pose is not collision-free")
    if not voxel_map.line_is_valid(start_xyz_m, points[0]):
        raise RuntimeError("actual start cannot connect to snapped A* path")
    if not voxel_map.line_is_valid(points[-1], goal_xyz_m):
        raise RuntimeError("snapped A* path cannot connect to actual goal")
    anchored = [list(start_xyz_m), *points, list(goal_xyz_m)]
    return _deduplicate(anchored).tolist()


def _deduplicate(points: list[list[float]]) -> np.ndarray:
    kept: list[np.ndarray] = []
    for point in points:
        value = np.asarray(point, dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError("B-spline path contains an invalid XYZ point")
        if not kept or np.linalg.norm(value - kept[-1]) > 1e-7:
            kept.append(value)
    if len(kept) < 2:
        raise ValueError("B-spline path requires at least two distinct points")
    return np.asarray(kept)


def _fit(
    points: list[list[float]], sample_step_m: float, smoothing_m: float,
    maximum_degree: int = 3,
) -> tuple[list[list[float]], int]:
    values = _deduplicate(points)
    edge_lengths = np.linalg.norm(np.diff(values, axis=0), axis=1)
    total = float(edge_lengths.sum())
    if total <= 1e-8:
        raise ValueError("B-spline path has zero length")
    parameter = np.concatenate(([0.0], np.cumsum(edge_lengths))) / total
    degree = min(maximum_degree, 3, len(values) - 1)
    smoothing = float(smoothing_m * smoothing_m * len(values))
    knots, _ = splprep(
        values.T, u=parameter, k=degree, s=smoothing, per=False,
    )
    sample_count = max(2, int(math.ceil(total / sample_step_m)) + 1)
    sampled = np.asarray(splev(np.linspace(0.0, 1.0, sample_count), knots)).T
    sampled[0], sampled[-1] = values[0], values[-1]
    return sampled.tolist(), degree


def _collision_free(points: list[list[float]], voxel_map: VoxelMap3D) -> bool:
    return bool(
        points
        and all(voxel_map.is_state_valid(point) for point in points)
        and all(
            voxel_map.line_is_valid(first, second)
            for first, second in zip(points, points[1:])
        )
    )


def _split_prefixes(points: list[list[float]], maximum_length_m: float) -> list[list[list[float]]]:
    values = _deduplicate(points)
    pieces: list[list[list[float]]] = []
    current = [values[0].tolist()]
    remaining_budget = maximum_length_m
    start = values[0]
    for original_end in values[1:]:
        end = original_end.copy()
        while True:
            distance = float(np.linalg.norm(end - start))
            if distance <= remaining_budget + 1e-9:
                current.append(end.tolist())
                remaining_budget -= distance
                start = end
                break
            ratio = remaining_budget / distance
            boundary = start + ratio * (end - start)
            current.append(boundary.tolist())
            pieces.append(current)
            current = [boundary.tolist()]
            start = boundary
            remaining_budget = maximum_length_m
    if len(current) >= 2:
        pieces.append(current)
    return pieces


def _try_piece(
    points: list[list[float]], voxel_map: VoxelMap3D, settings: BsplineSettings,
    allow_linear: bool,
) -> tuple[list[list[float]], int, float] | None:
    valid_candidates: list[tuple[float, float, float, list[list[float]], int]] = []
    for smoothing in settings.smoothing_candidates:
        try:
            sampled, degree = _fit(points, settings.sample_step_m, smoothing, 3)
        except (TypeError, ValueError):
            continue
        if _collision_free(sampled, voxel_map):
            clearances = [voxel_map.clearance(point) for point in sampled]
            preferred = voxel_map.profile.preferred_esdf_distance_m
            mean_clearance_deficit = sum(
                max(0.0, preferred - value) ** 2 for value in clearances
            ) / len(clearances)
            length = sum(
                math.dist(first, second)
                for first, second in zip(sampled, sampled[1:])
            )
            valid_candidates.append(
                (mean_clearance_deficit, length, -smoothing, sampled, degree)
            )
    if valid_candidates:
        _, _, negative_smoothing, sampled, degree = min(
            valid_candidates, key=lambda item: item[:3]
        )
        return sampled, degree, -negative_smoothing
    if allow_linear:
        sampled, degree = _fit(points, settings.sample_step_m, 0.0, 1)
        if _collision_free(sampled, voxel_map):
            return sampled, degree, 0.0
    return None


def plan_collision_checked_bspline(
    astar_points_xyz_m: list[list[float]],
    voxel_map: VoxelMap3D,
    settings: BsplineSettings,
) -> BsplinePath3D:
    """Try a full curve, then validated pieces of the same locked A* path."""
    full = _try_piece(astar_points_xyz_m, voxel_map, settings, allow_linear=False)
    if full is not None:
        sampled, degree, smoothing = full
        return BsplinePath3D(
            sampled, "full_bspline", degree, smoothing, 1,
            min(voxel_map.clearance(point) for point in sampled),
        )

    combined: list[list[float]] = []
    degrees: list[int] = []
    smoothing_values: list[float] = []
    pieces = _split_prefixes(astar_points_xyz_m, settings.conditional_prefix_length_m)
    for piece in pieces:
        fitted = _try_piece(piece, voxel_map, settings, allow_linear=True)
        if fitted is None:
            raise RuntimeError("same-goal conditional B-spline prefix failed collision check")
        sampled, degree, smoothing = fitted
        combined.extend(sampled if not combined else sampled[1:])
        degrees.append(degree)
        smoothing_values.append(smoothing)
    if not _collision_free(combined, voxel_map):
        raise RuntimeError("combined conditional B-spline failed final collision check")
    return BsplinePath3D(
        combined, "conditional_same_goal_prefix_bspline", min(degrees),
        max(smoothing_values), len(pieces),
        min(voxel_map.clearance(point) for point in combined),
    )
