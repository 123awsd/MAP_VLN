"""Versioned FALCON raw occupancy, inflated occupancy, and ESDF snapshots."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .planning_contract import (
    MapIdentity,
    OccupancyState,
    PlannerProfile,
    stable_hash,
)


class VoxelMap3D:
    """Immutable z,y,x arrays with one shared collision predicate."""

    def __init__(
        self,
        raw_occupancy_zyx: np.ndarray,
        esdf_zyx_m: np.ndarray,
        origin_xyz_m: Iterable[float],
        resolution_m: float,
        profile: PlannerProfile,
        identity: MapIdentity,
    ):
        raw = np.asarray(raw_occupancy_zyx, dtype=np.int8)
        esdf = np.asarray(esdf_zyx_m, dtype=np.float32)
        if raw.ndim != 3 or raw.shape != esdf.shape:
            raise ValueError("raw occupancy and ESDF must be same-shaped 3-D arrays")
        allowed = {
            int(OccupancyState.UNKNOWN), int(OccupancyState.FREE), int(OccupancyState.OCCUPIED)
        }
        if not set(np.unique(raw).tolist()) <= allowed:
            raise ValueError("raw occupancy contains values outside UNKNOWN/FREE/OCCUPIED")
        if identity.planner_profile_hash != profile.profile_hash:
            raise ValueError("snapshot planner profile hash does not match requested profile")
        self.raw = raw
        self.esdf = esdf
        self.origin = np.asarray(list(origin_xyz_m), dtype=np.float64)
        self.resolution = float(resolution_m)
        self.profile = profile
        self.identity = identity
        # FALCON's B-spline collision constraint is an ESDF threshold. Unknown
        # remains blocked even if its distance to occupied geometry is large.
        self.inflated_free = (
            (self.raw == int(OccupancyState.FREE))
            & (self.esdf + 1e-6 >= self.profile.minimum_esdf_distance_m)
        )

    @classmethod
    def load(cls, directory: Path, profile: PlannerProfile) -> "VoxelMap3D":
        metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
        arrays = np.load(directory / "voxel_map.npz")
        identity = MapIdentity(**metadata["identity"])
        return cls(
            arrays["raw_occupancy_zyx"], arrays["esdf_zyx_m"],
            metadata["origin_xyz_m"], metadata["resolution_m"], profile, identity,
        )

    @property
    def shape_xyz(self) -> tuple[int, int, int]:
        return self.raw.shape[2], self.raw.shape[1], self.raw.shape[0]

    def world_to_voxel(self, xyz: Iterable[float]) -> tuple[int, int, int]:
        index = np.floor((np.asarray(list(xyz), dtype=float) - self.origin) / self.resolution).astype(int)
        return int(index[0]), int(index[1]), int(index[2])

    def voxel_to_world(self, index_xyz: Iterable[int]) -> list[float]:
        return (self.origin + (np.asarray(list(index_xyz), dtype=float) + 0.5) * self.resolution).tolist()

    def in_bounds(self, index_xyz: Iterable[int]) -> bool:
        x, y, z = [int(value) for value in index_xyz]
        nx, ny, nz = self.shape_xyz
        return 0 <= x < nx and 0 <= y < ny and 0 <= z < nz

    def raw_state(self, xyz: Iterable[float]) -> OccupancyState:
        index = self.world_to_voxel(xyz)
        if not self.in_bounds(index):
            return OccupancyState.UNKNOWN
        x, y, z = index
        return OccupancyState(int(self.raw[z, y, x]))

    def clearance(self, xyz: Iterable[float]) -> float:
        index = self.world_to_voxel(xyz)
        if not self.in_bounds(index):
            return 0.0
        x, y, z = index
        return float(self.esdf[z, y, x])

    def is_state_valid(self, xyz: Iterable[float]) -> bool:
        index = self.world_to_voxel(xyz)
        if not self.in_bounds(index):
            return False
        x, y, z = index
        return bool(self.inflated_free[z, y, x])

    def nearest_valid(self, xyz: Iterable[float], radius_m: float | None = None) -> list[float] | None:
        radius = self.profile.nearest_free_radius_m if radius_m is None else float(radius_m)
        center = self.world_to_voxel(xyz)
        cells = int(math.ceil(radius / self.resolution))
        best = None
        best_distance = math.inf
        for dz in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    distance = dx * dx + dy * dy + dz * dz
                    if distance > cells * cells:
                        continue
                    candidate = center[0] + dx, center[1] + dy, center[2] + dz
                    if not self.in_bounds(candidate):
                        continue
                    x, y, z = candidate
                    if self.inflated_free[z, y, x] and distance < best_distance:
                        best, best_distance = candidate, distance
        return None if best is None else self.voxel_to_world(best)

    def line_is_valid(self, start_xyz: Iterable[float], end_xyz: Iterable[float]) -> bool:
        start = np.asarray(list(start_xyz), dtype=float)
        end = np.asarray(list(end_xyz), dtype=float)
        distance = float(np.linalg.norm(end - start))
        samples = max(1, int(math.ceil(distance / (0.5 * self.resolution))))
        return all(
            self.is_state_valid(start + (end - start) * (index / samples))
            for index in range(samples + 1)
        )

    def line_of_sight(
        self, start_xyz: Iterable[float], end_xyz: Iterable[float],
        endpoint_allowance_m: float = 0.25,
    ) -> bool:
        """Require observed FREE until the target's occupied endpoint neighborhood."""
        start = np.asarray(list(start_xyz), dtype=float)
        end = np.asarray(list(end_xyz), dtype=float)
        distance = float(np.linalg.norm(end - start))
        checked_distance = max(0.0, distance - float(endpoint_allowance_m))
        samples = max(1, int(math.ceil(checked_distance / (0.5 * self.resolution))))
        if distance <= 1e-9:
            return self.raw_state(start) == OccupancyState.FREE
        return all(
            self.raw_state(start + (end - start) * ((index / samples) * checked_distance / distance))
            == OccupancyState.FREE
            for index in range(samples + 1)
        )

    def planning_state_signature(self) -> str:
        bands = np.digitize(self.esdf, self.profile.esdf_version_bands_m).astype(np.uint8)
        digest = hashlib.sha256()
        digest.update(self.raw.tobytes(order="C"))
        digest.update(self.inflated_free.tobytes(order="C"))
        digest.update(bands.tobytes(order="C"))
        digest.update(self.profile.profile_hash.encode("ascii"))
        return digest.hexdigest()

    def conservative_coarse_free(self) -> tuple[np.ndarray, float]:
        ratio_float = self.profile.coarse_resolution_m / self.resolution
        ratio = int(round(ratio_float))
        if ratio < 1 or not math.isclose(ratio_float, ratio, abs_tol=1e-6):
            raise ValueError("coarse resolution must be an integer multiple of snapshot resolution")
        z, y, x = self.inflated_free.shape
        cz, cy, cx = z // ratio, y // ratio, x // ratio
        trimmed = self.inflated_free[:cz * ratio, :cy * ratio, :cx * ratio]
        coarse = trimmed.reshape(cz, ratio, cy, ratio, cx, ratio).all(axis=(1, 3, 5))
        return coarse, self.resolution * ratio


def snapshot_metadata_hash(metadata: dict) -> str:
    value = dict(metadata)
    value.pop("identity", None)
    return stable_hash(value)
