"""Dependency-free VLN trajectory metrics for the common comparison adapter."""

from __future__ import annotations

import math
from typing import Callable, Sequence


Point = Sequence[float]
Distance = Callable[[Point, Point], float]


def euclidean(a: Point, b: Point) -> float:
    return math.dist(tuple(float(value) for value in a), tuple(float(value) for value in b))


def path_length(path: Sequence[Point], distance: Distance = euclidean) -> float:
    return sum(distance(a, b) for a, b in zip(path, path[1:]))


def dtw(path: Sequence[Point], reference: Sequence[Point], distance: Distance = euclidean) -> float:
    if not path or not reference:
        raise ValueError("trajectory and reference must both be non-empty")
    previous = [math.inf] * (len(reference) + 1)
    previous[0] = 0.0
    for point in path:
        current = [math.inf] * (len(reference) + 1)
        for index, target in enumerate(reference, start=1):
            current[index] = distance(point, target) + min(
                current[index - 1], previous[index], previous[index - 1]
            )
        previous = current
    return previous[-1]


def evaluate_trajectory(
    trajectory: Sequence[Point],
    reference: Sequence[Point],
    *,
    shortest_path_length: float | None = None,
    success_distance: float = 3.0,
    distance: Distance = euclidean,
) -> dict[str, float | bool]:
    """Compute common NE/OSR/SR/SPL/nDTW metrics.

    Pass Habitat geodesic ``distance`` and the episode shortest-path length for
    official R2R-CE evaluation. Euclidean distance is suitable only for unit
    tests and controlled already-navigable polylines.
    """
    if not trajectory or not reference:
        raise ValueError("trajectory and reference must both be non-empty")
    goal = reference[-1]
    navigation_error = distance(trajectory[-1], goal)
    oracle_error = min(distance(point, goal) for point in trajectory)
    success = navigation_error <= success_distance
    oracle_success = oracle_error <= success_distance
    actual_length = path_length(trajectory, distance)
    optimal_length = (
        path_length(reference, distance)
        if shortest_path_length is None
        else float(shortest_path_length)
    )
    spl = float(success) * optimal_length / max(actual_length, optimal_length, 1e-9)
    normalized_dtw = math.exp(-dtw(trajectory, reference, distance) / (len(reference) * success_distance))
    return {
        "NE": navigation_error,
        "OSR": oracle_success,
        "SR": success,
        "SPL": spl,
        "nDTW": normalized_dtw,
        "trajectory_length": actual_length,
    }
