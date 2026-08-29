"""Shared contracts for coarse and authoritative three-dimensional planning."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from enum import IntEnum
from pathlib import Path
from typing import Any

import yaml


class OccupancyState(IntEnum):
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100


def stable_hash(value: Any) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class PlannerProfile:
    unknown_is_blocked: bool = True
    vehicle_radius_xy_m: float = 0.25
    vehicle_radius_z_m: float = 0.15
    safety_margin_m: float = 0.10
    minimum_esdf_distance_m: float = 0.35
    preferred_esdf_distance_m: float = 0.70
    clearance_cost_weight: float = 0.0
    coarse_resolution_m: float = 0.20
    connectivity: int = 26
    nearest_free_radius_m: float = 0.40
    esdf_version_bands_m: tuple[float, ...] = (0.35, 0.50, 0.70, 0.80)
    maximum_expansions: int = 2_000_000

    @classmethod
    def load(cls, path: Path) -> "PlannerProfile":
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        occupancy = value.get("occupancy", {})
        vehicle = value.get("vehicle", {})
        planning = value.get("planning", {})
        if (
            int(occupancy.get("unknown_value", -1)) != OccupancyState.UNKNOWN
            or int(occupancy.get("free_value", 0)) != OccupancyState.FREE
            or int(occupancy.get("occupied_value", 100)) != OccupancyState.OCCUPIED
        ):
            raise ValueError("planning config occupancy values do not match the shared contract")
        profile = cls(
            unknown_is_blocked=bool(occupancy.get("unknown_is_blocked", True)),
            vehicle_radius_xy_m=float(vehicle.get("radius_xy_m", 0.25)),
            vehicle_radius_z_m=float(vehicle.get("radius_z_m", 0.15)),
            safety_margin_m=float(vehicle.get("safety_margin_m", 0.10)),
            minimum_esdf_distance_m=float(planning.get("minimum_esdf_distance_m", 0.35)),
            preferred_esdf_distance_m=float(planning.get("preferred_esdf_distance_m", 0.70)),
            clearance_cost_weight=float(planning.get("clearance_cost_weight", 0.0)),
            coarse_resolution_m=float(planning.get("coarse_resolution_m", 0.20)),
            connectivity=int(planning.get("connectivity", 26)),
            nearest_free_radius_m=float(planning.get("nearest_free_radius_m", 0.40)),
            esdf_version_bands_m=tuple(
                float(item) for item in planning.get("esdf_version_bands_m", (0.35, 0.50, 0.70, 0.80))
            ),
            maximum_expansions=int(planning.get("maximum_expansions", 2_000_000)),
        )
        profile.validate()
        return profile

    def validate(self) -> None:
        if not self.unknown_is_blocked:
            raise ValueError("stage-two UAV execution requires unknown_is_blocked=true")
        if self.connectivity not in (6, 18, 26):
            raise ValueError("3-D connectivity must be 6, 18, or 26")
        if min(
            self.vehicle_radius_xy_m,
            self.vehicle_radius_z_m,
            self.safety_margin_m,
            self.minimum_esdf_distance_m,
        ) < 0:
            raise ValueError("vehicle dimensions and hard clearance cannot be negative")
        if self.preferred_esdf_distance_m <= 0 or self.coarse_resolution_m <= 0:
            raise ValueError("preferred clearance and coarse resolution must be positive")
        if self.clearance_cost_weight < 0:
            raise ValueError("clearance cost weight cannot be negative")
        required = max(self.vehicle_radius_xy_m, self.vehicle_radius_z_m) + self.safety_margin_m
        if self.minimum_esdf_distance_m + 1e-9 < required:
            raise ValueError("hard ESDF clearance must cover vehicle radius plus safety margin")
        if self.preferred_esdf_distance_m < self.minimum_esdf_distance_m:
            raise ValueError("preferred ESDF clearance cannot be below hard clearance")
        if tuple(sorted(self.esdf_version_bands_m)) != self.esdf_version_bands_m:
            raise ValueError("ESDF version bands must be sorted")

    @property
    def profile_hash(self) -> str:
        return stable_hash(asdict(self))


@dataclass(frozen=True)
class MapIdentity:
    map_epoch_uuid: str
    geometry_map_version: int
    semantic_map_version: int
    snapshot_content_hash: str
    occupancy_semantics_hash: str
    planner_profile_hash: str

    @property
    def geometry_cache_namespace(self) -> tuple[str, int, str, str]:
        return (
            self.map_epoch_uuid,
            self.geometry_map_version,
            self.occupancy_semantics_hash,
            self.planner_profile_hash,
        )


@dataclass(frozen=True)
class Path3D:
    points_xyz_m: list[list[float]]
    length_m: float
    vertical_distance_m: float
    minimum_clearance_m: float
    map_identity: MapIdentity
    adjusted_start_xyz_m: list[float]
    adjusted_goal_xyz_m: list[float]
    expanded_nodes: int

    def to_json(self) -> dict[str, Any]:
        return {
            "status": "success",
            "map_epoch_uuid": self.map_identity.map_epoch_uuid,
            "geometry_map_version": self.map_identity.geometry_map_version,
            "planner_profile_hash": self.map_identity.planner_profile_hash,
            "points_xyz_m": self.points_xyz_m,
            "length_m": self.length_m,
            "vertical_distance_m": self.vertical_distance_m,
            "minimum_clearance_m": self.minimum_clearance_m,
            "adjusted_start_xyz_m": self.adjusted_start_xyz_m,
            "adjusted_goal_xyz_m": self.adjusted_goal_xyz_m,
            "expanded_nodes": self.expanded_nodes,
        }


OCCUPANCY_SEMANTICS = {
    "unknown": int(OccupancyState.UNKNOWN),
    "free": int(OccupancyState.FREE),
    "occupied": int(OccupancyState.OCCUPIED),
    "unknown_is_blocked": True,
    "coarse_reduction": "free_only_if_every_fine_voxel_is_inflated_free",
}
OCCUPANCY_SEMANTICS_HASH = stable_hash(OCCUPANCY_SEMANTICS)
