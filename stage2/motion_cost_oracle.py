"""Version-bound cached motion paths for stage-two joint optimization."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Iterable

from .astar_3d import AstarFailure, CoarseAstar3D
from .io_utils import atomic_json
from .planning_contract import MapIdentity, Path3D


class MotionCostOracle:
    def __init__(
        self, planner: CoarseAstar3D, cache_path: Path | None = None,
        save_interval: int = 1,
    ):
        self.planner = planner
        self.voxel_map = planner.voxel_map
        self.identity = self.voxel_map.identity
        self.cache_path = cache_path
        self.entries: dict[str, dict] = {}
        self.cache_hits = 0
        self.cache_misses = 0
        self.save_interval = max(1, int(save_interval))
        self._dirty_entries = 0
        if cache_path is not None and cache_path.is_file():
            value = json.loads(cache_path.read_text(encoding="utf-8"))
            if value.get("format") == "pre_map_vln.motion_cost_cache.v1":
                self.entries = dict(value.get("entries", {}))

    @property
    def resolution(self) -> float:
        return self.voxel_map.resolution

    def is_state_valid(self, xyz: Iterable[float]) -> bool:
        return self.voxel_map.is_state_valid(xyz)

    def line_of_sight(self, start_xyz: Iterable[float], end_xyz: Iterable[float]) -> bool:
        return self.voxel_map.line_of_sight(start_xyz, end_xyz)

    def _key(self, start_xyz: Iterable[float], goal_xyz: Iterable[float]) -> str:
        namespace = self.identity.geometry_cache_namespace
        start = self.planner.world_to_cell(start_xyz)
        goal = self.planner.world_to_cell(goal_xyz)
        payload = [*namespace, start, goal]
        return hashlib.sha256(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()

    def _save(self) -> None:
        if self.cache_path is None:
            return
        atomic_json(self.cache_path, {
            "format": "pre_map_vln.motion_cost_cache.v1",
            "map_identity": self.identity.__dict__,
            "entries": self.entries,
        })
        self._dirty_entries = 0

    def flush(self) -> None:
        if self._dirty_entries:
            self._save()

    def path(self, start_xyz: Iterable[float], goal_xyz: Iterable[float]) -> Path3D | None:
        start_values, goal_values = list(start_xyz), list(goal_xyz)
        key = self._key(start_values, goal_values)
        reverse_key = self._key(goal_values, start_values)
        reverse = False
        entry = self.entries.get(key)
        if entry is None:
            entry = self.entries.get(reverse_key)
            reverse = entry is not None
        if entry is not None:
            self.cache_hits += 1
            if entry["status"] != "success":
                return None
            points = list(reversed(entry["points_xyz_m"])) if reverse else entry["points_xyz_m"]
            return Path3D(
                points_xyz_m=points,
                length_m=float(entry["length_m"]),
                vertical_distance_m=float(entry["vertical_distance_m"]),
                minimum_clearance_m=float(entry["minimum_clearance_m"]),
                map_identity=self.identity,
                adjusted_start_xyz_m=list(points[0]), adjusted_goal_xyz_m=list(points[-1]),
                expanded_nodes=int(entry.get("expanded_nodes", 0)),
            )
        self.cache_misses += 1
        result = self.planner.plan(start_values, goal_values)
        if isinstance(result, AstarFailure):
            self.entries[key] = {
                "status": result.status, "reason": result.reason,
                "expanded_nodes": result.expanded_nodes,
            }
            self._dirty_entries += 1
            if self._dirty_entries >= self.save_interval:
                self._save()
            return None
        self.entries[key] = result.to_json()
        self._dirty_entries += 1
        if self._dirty_entries >= self.save_interval:
            self._save()
        return result

    def cost(self, start_xyz: Iterable[float], goal_xyz: Iterable[float]) -> float:
        path = self.path(start_xyz, goal_xyz)
        return float("inf") if path is None else path.length_m

    def statistics(self) -> dict:
        self.flush()
        return {
            "cache_hits": self.cache_hits, "cache_misses": self.cache_misses,
            "entry_count": len(self.entries),
            "map_epoch_uuid": self.identity.map_epoch_uuid,
            "geometry_map_version": self.identity.geometry_map_version,
            "planner_profile_hash": self.identity.planner_profile_hash,
        }
