"""Conservative coarse 3-D A* over a versioned FALCON voxel snapshot."""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .planning_contract import Path3D
from .voxel_map_3d import VoxelMap3D


@dataclass(frozen=True)
class AstarFailure:
    status: str
    reason: str
    expanded_nodes: int


def neighbor_offsets(connectivity: int) -> list[tuple[int, int, int, float]]:
    result = []
    for dz, dy, dx in itertools.product((-1, 0, 1), repeat=3):
        nonzero = int(dx != 0) + int(dy != 0) + int(dz != 0)
        if nonzero == 0:
            continue
        if connectivity == 6 and nonzero != 1:
            continue
        if connectivity == 18 and nonzero == 3:
            continue
        result.append((dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz)))
    return result


def swept_offsets(dx: int, dy: int, dz: int) -> list[tuple[int, int, int]]:
    """Intermediate cells required to prevent edge/corner clipping."""
    axes = [(0, dx), (1, dy), (2, dz)]
    active = [(axis, value) for axis, value in axes if value]
    if len(active) <= 1:
        return []
    result = []
    for count in range(1, len(active)):
        for subset in itertools.combinations(active, count):
            offset = [0, 0, 0]
            for axis, value in subset:
                offset[axis] = value
            result.append(tuple(offset))
    return result


class CoarseAstar3D:
    def __init__(self, voxel_map: VoxelMap3D):
        self.voxel_map = voxel_map
        self.free_zyx, self.resolution = voxel_map.conservative_coarse_free()
        self.origin = voxel_map.origin.copy()
        self.offsets = neighbor_offsets(voxel_map.profile.connectivity)
        self.swept_by_delta = {
            (dx, dy, dz): swept_offsets(dx, dy, dz)
            for dx, dy, dz, _ in self.offsets
        }
        ratio = int(round(self.resolution / voxel_map.resolution))
        z, y, x = self.free_zyx.shape
        trimmed = voxel_map.esdf[:z * ratio, :y * ratio, :x * ratio]
        self.minimum_clearance_zyx = trimmed.reshape(
            z, ratio, y, ratio, x, ratio
        ).min(axis=(1, 3, 5))

    @property
    def shape_xyz(self) -> tuple[int, int, int]:
        return self.free_zyx.shape[2], self.free_zyx.shape[1], self.free_zyx.shape[0]

    def world_to_cell(self, xyz: Iterable[float]) -> tuple[int, int, int]:
        value = np.floor((np.asarray(list(xyz), dtype=float) - self.origin) / self.resolution).astype(int)
        return int(value[0]), int(value[1]), int(value[2])

    def cell_to_world(self, cell: Iterable[int]) -> list[float]:
        return (self.origin + (np.asarray(list(cell), dtype=float) + 0.5) * self.resolution).tolist()

    def in_bounds(self, cell: tuple[int, int, int]) -> bool:
        x, y, z = cell
        nx, ny, nz = self.shape_xyz
        return 0 <= x < nx and 0 <= y < ny and 0 <= z < nz

    def is_free(self, cell: tuple[int, int, int]) -> bool:
        if not self.in_bounds(cell):
            return False
        x, y, z = cell
        return bool(self.free_zyx[z, y, x])

    def nearest_free(self, xyz: Iterable[float], radius_m: float | None = None) -> tuple[int, int, int] | None:
        radius = self.voxel_map.profile.nearest_free_radius_m if radius_m is None else float(radius_m)
        center = self.world_to_cell(xyz)
        cells = int(math.ceil(radius / self.resolution))
        best, best_distance = None, math.inf
        for dz in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    distance = dx * dx + dy * dy + dz * dz
                    if distance > cells * cells:
                        continue
                    candidate = center[0] + dx, center[1] + dy, center[2] + dz
                    if self.is_free(candidate) and distance < best_distance:
                        best, best_distance = candidate, distance
        return best

    def transition_is_free(
        self, current: tuple[int, int, int], delta: tuple[int, int, int]
    ) -> bool:
        dx, dy, dz = delta
        neighbor = current[0] + dx, current[1] + dy, current[2] + dz
        if not self.is_free(neighbor):
            return False
        return all(
            self.is_free((current[0] + ox, current[1] + oy, current[2] + oz))
            for ox, oy, oz in self.swept_by_delta[(dx, dy, dz)]
        )

    def transition_cost(self, neighbor: tuple[int, int, int], geometric_cost: float) -> float:
        """Add a bounded soft clearance preference without changing reachability."""
        preferred = self.voxel_map.profile.preferred_esdf_distance_m
        weight = self.voxel_map.profile.clearance_cost_weight
        if weight <= 0.0 or preferred <= 0.0:
            return geometric_cost
        x, y, z = neighbor
        clearance = float(self.minimum_clearance_zyx[z, y, x])
        deficit = max(0.0, min(1.0, (preferred - clearance) / preferred))
        return geometric_cost * (1.0 + weight * deficit * deficit)

    def plan(self, start_xyz: Iterable[float], goal_xyz: Iterable[float]) -> Path3D | AstarFailure:
        start = self.nearest_free(start_xyz)
        if start is None:
            return AstarFailure("invalid_start", "no inflated-free start voxel within snap radius", 0)
        goal = self.nearest_free(goal_xyz)
        if goal is None:
            return AstarFailure("invalid_goal", "no inflated-free goal voxel within snap radius", 0)
        if start == goal:
            point = self.cell_to_world(start)
            return Path3D(
                [point], 0.0, 0.0, self.voxel_map.clearance(point), self.voxel_map.identity,
                point, point, 0,
            )

        g_score = np.full(self.free_zyx.shape, np.inf, dtype=np.float32)
        closed = np.zeros(self.free_zyx.shape, dtype=bool)
        parent: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        g_score[start[2], start[1], start[0]] = 0.0
        queue = [(math.dist(start, goal), 0.0, start)]
        expanded = 0
        maximum = self.voxel_map.profile.maximum_expansions
        while queue:
            _, cost, current = heapq.heappop(queue)
            x, y, z = current
            if closed[z, y, x] or cost > float(g_score[z, y, x]) + 1e-6:
                continue
            closed[z, y, x] = True
            expanded += 1
            if current == goal:
                cells = [current]
                while cells[-1] != start:
                    cells.append(parent[cells[-1]])
                cells.reverse()
                points = [self.cell_to_world(cell) for cell in cells]
                # Coarse cells were conservatively reduced from fine validity;
                # still audit every continuous segment against the fine map.
                if not all(
                    self.voxel_map.line_is_valid(first, second)
                    for first, second in zip(points, points[1:])
                ):
                    return AstarFailure(
                        "collision_recheck_failed",
                        "coarse path failed fine inflated-occupancy recheck", expanded,
                    )
                lengths = [math.dist(first, second) for first, second in zip(points, points[1:])]
                clearances = [self.voxel_map.clearance(point) for point in points]
                return Path3D(
                    points_xyz_m=points,
                    length_m=float(sum(lengths)),
                    vertical_distance_m=float(sum(abs(second[2] - first[2]) for first, second in zip(points, points[1:]))),
                    minimum_clearance_m=float(min(clearances)),
                    map_identity=self.voxel_map.identity,
                    adjusted_start_xyz_m=points[0],
                    adjusted_goal_xyz_m=points[-1],
                    expanded_nodes=expanded,
                )
            if expanded >= maximum:
                return AstarFailure("expansion_limit", f"exceeded {maximum} expanded nodes", expanded)
            for dx, dy, dz, step in self.offsets:
                if not self.transition_is_free(current, (dx, dy, dz)):
                    continue
                neighbor = x + dx, y + dy, z + dz
                nx, ny, nz = neighbor
                candidate = cost + self.transition_cost(neighbor, step * self.resolution)
                if candidate + 1e-6 >= float(g_score[nz, ny, nx]):
                    continue
                g_score[nz, ny, nx] = candidate
                parent[neighbor] = current
                heuristic = math.dist(neighbor, goal) * self.resolution
                heapq.heappush(queue, (candidate + heuristic, candidate, neighbor))
        return AstarFailure("no_path", "inflated FREE component does not connect start and goal", expanded)
