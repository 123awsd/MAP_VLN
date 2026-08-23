"""Occupancy-grid geometry and deterministic A* for semantic mission planning."""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class GridPath:
    points_xy_m: list[list[float]]
    length_m: float


def line_cells(start: tuple[int, int], end: tuple[int, int]) -> Iterable[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def inflate(mask: np.ndarray, radius: int) -> np.ndarray:
    result = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            y0, y1 = max(0, dy), min(mask.shape[0], mask.shape[0] + dy)
            x0, x1 = max(0, dx), min(mask.shape[1], mask.shape[1] + dx)
            result[y0:y1, x0:x1] |= mask[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    return result


class OccupancyGrid:
    def __init__(self, grid: np.ndarray, origin_xy: Iterable[float], resolution: float, inflation_m: float = 0.10):
        if grid.ndim != 2:
            raise ValueError("occupancy grid must be 2-D")
        self.grid = np.asarray(grid, dtype=np.int8)
        self.origin = np.asarray(list(origin_xy), dtype=np.float64)
        self.resolution = float(resolution)
        radius = max(0, int(math.ceil(inflation_m / self.resolution)))
        self.blocked = inflate(self.grid != 0, radius)
        self._path_cache: dict[tuple[tuple[int, int], tuple[int, int]], GridPath | None] = {}

    @classmethod
    def load(cls, prefix: Path, inflation_m: float = 0.10) -> "OccupancyGrid":
        grid = np.load(prefix.with_suffix(".npy"))
        metadata = json.loads(prefix.with_suffix(".json").read_text(encoding="utf-8"))
        return cls(grid, metadata["origin_xy_m"], metadata["resolution_m"], inflation_m=inflation_m)

    def world_to_cell(self, point_xy: Iterable[float]) -> tuple[int, int]:
        value = np.floor((np.asarray(list(point_xy), dtype=np.float64) - self.origin) / self.resolution).astype(int)
        return int(value[0]), int(value[1])

    def cell_to_world(self, cell: tuple[int, int]) -> list[float]:
        return (self.origin + (np.asarray(cell, dtype=np.float64) + 0.5) * self.resolution).tolist()

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]

    def is_free(self, point_xy: Iterable[float]) -> bool:
        cell = self.world_to_cell(point_xy)
        return self.in_bounds(cell) and not bool(self.blocked[cell[1], cell[0]])

    def nearest_free(self, point_xy: Iterable[float], max_radius_m: float = 1.5) -> list[float] | None:
        center = self.world_to_cell(point_xy)
        max_cells = int(math.ceil(max_radius_m / self.resolution))
        best = None
        best_distance = float("inf")
        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                cell = (center[0] + dx, center[1] + dy)
                if not self.in_bounds(cell) or self.blocked[cell[1], cell[0]]:
                    continue
                distance = dx * dx + dy * dy
                if distance < best_distance:
                    best, best_distance = cell, distance
        return None if best is None else self.cell_to_world(best)

    def line_is_free(self, start_xy: Iterable[float], end_xy: Iterable[float], allow_endpoint_cells: int = 0) -> bool:
        cells = list(line_cells(self.world_to_cell(start_xy), self.world_to_cell(end_xy)))
        checked = cells[:-allow_endpoint_cells] if allow_endpoint_cells > 0 else cells
        return all(self.in_bounds(cell) and not self.blocked[cell[1], cell[0]] for cell in checked)

    def astar(self, start_xy: Iterable[float], goal_xy: Iterable[float]) -> GridPath | None:
        start_world = self.nearest_free(start_xy, max_radius_m=0.8)
        goal_world = self.nearest_free(goal_xy, max_radius_m=0.4)
        if start_world is None or goal_world is None:
            return None
        start, goal = self.world_to_cell(start_world), self.world_to_cell(goal_world)
        key = (start, goal)
        reverse_key = (goal, start)
        if key in self._path_cache:
            return self._path_cache[key]
        if reverse_key in self._path_cache:
            reverse = self._path_cache[reverse_key]
            return None if reverse is None else GridPath(list(reversed(reverse.points_xy_m)), reverse.length_m)
        if start == goal:
            result = GridPath([self.cell_to_world(start)], 0.0)
            self._path_cache[key] = result
            return result

        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
        ]
        queue = [(0.0, 0.0, start)]
        costs = {start: 0.0}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > costs.get(current, float("inf")) + 1e-9:
                continue
            if current == goal:
                cells = [current]
                while cells[-1] != start:
                    cells.append(parent[cells[-1]])
                cells.reverse()
                points = [self.cell_to_world(cell) for cell in cells]
                result = GridPath(points, cost * self.resolution)
                self._path_cache[key] = result
                return result
            for dx, dy, step in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.in_bounds(neighbor) or self.blocked[neighbor[1], neighbor[0]]:
                    continue
                if dx and dy:
                    if self.blocked[current[1], neighbor[0]] or self.blocked[neighbor[1], current[0]]:
                        continue
                candidate = cost + step
                if candidate + 1e-9 >= costs.get(neighbor, float("inf")):
                    continue
                costs[neighbor] = candidate
                parent[neighbor] = current
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(queue, (candidate + heuristic, candidate, neighbor))
        self._path_cache[key] = None
        return None
