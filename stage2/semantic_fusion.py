"""Multi-frame 3-D semantic fusion and novel-object scene-graph materialization."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Any


@dataclass
class SemanticTrack:
    id: str
    label: str
    center: list[float]
    size: list[float]
    score: float
    support_count: int
    first_frame: int
    last_frame: int
    associated_object_id: str | None = None
    observations: list[dict[str, Any]] = field(default_factory=list)

    @property
    def confirmed(self) -> bool:
        return self.associated_object_id is not None or self.support_count >= 2

    def update(self, item: dict[str, Any], frame_index: int) -> None:
        weight = 1.0 / (self.support_count + 1)
        self.center = [old * (1.0 - weight) + float(new) * weight for old, new in zip(self.center, item["center"])]
        self.size = [old * (1.0 - weight) + float(new) * weight for old, new in zip(self.size, item["size"])]
        self.score = max(self.score, float(item["score"]))
        self.support_count += 1
        self.last_frame = frame_index
        self.observations.append({"frame_index": frame_index, "score": item["score"], "center": item["center"]})
        self.observations = self.observations[-12:]

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id, "label": self.label,
            "center": [round(value, 4) for value in self.center],
            "size": [round(value, 4) for value in self.size],
            "score": round(self.score, 6), "support_count": self.support_count,
            "first_frame": self.first_frame, "last_frame": self.last_frame,
            "associated_object_id": self.associated_object_id,
            "source": "pre_map_association" if self.associated_object_id else "online_novel",
            "confirmed": self.confirmed,
        }


class OnlineSemanticFusion:
    def __init__(self, association_radius_m: float = 0.75, minimum_novel_support: int = 2):
        self.association_radius_m = float(association_radius_m)
        self.minimum_novel_support = int(minimum_novel_support)
        self.tracks: dict[str, SemanticTrack] = {}
        self._novel_sequence = 0

    def _track_for(self, item: dict[str, Any]) -> SemanticTrack | None:
        known_id = item.get("associated_object_id") if item.get("confirmed") else None
        if known_id:
            return self.tracks.get(f"known:{known_id}")
        options = [
            track for track in self.tracks.values()
            if track.associated_object_id is None
            and track.label == item["label"]
            and math.dist(track.center, item["center"]) <= self.association_radius_m
        ]
        return min(options, key=lambda track: math.dist(track.center, item["center"]), default=None)

    def update(self, projected_items: list[dict[str, Any]], frame_index: int) -> list[dict[str, Any]]:
        for item in projected_items:
            known_id = item.get("associated_object_id") if item.get("confirmed") else None
            track = self._track_for(item)
            if track is None:
                if known_id:
                    track_id = f"known:{known_id}"
                else:
                    track_id = f"online_{self._novel_sequence:04d}"
                    self._novel_sequence += 1
                track = SemanticTrack(
                    id=track_id, label=item["label"], center=list(map(float, item["center"])),
                    size=list(map(float, item["size"])), score=float(item["score"]),
                    support_count=1, first_frame=frame_index, last_frame=frame_index,
                    associated_object_id=known_id,
                    observations=[{"frame_index": frame_index, "score": item["score"], "center": item["center"]}],
                )
                self.tracks[track_id] = track
            else:
                track.update(item, frame_index)
        return self.snapshot(confirmed_only=True)

    def snapshot(self, confirmed_only: bool = True) -> list[dict[str, Any]]:
        values = []
        for track in self.tracks.values():
            item = track.as_dict()
            item["confirmed"] = bool(
                track.associated_object_id or track.support_count >= self.minimum_novel_support
            )
            values.append(item)
        if confirmed_only:
            values = [item for item in values if item["confirmed"]]
        return sorted(values, key=lambda item: item["id"])

    def materialize_scene_graph(self, scene_graph: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
        result = copy.deepcopy(scene_graph)
        existing_ids = {obj["id"] for room in result.get("rooms", []) for obj in room.get("objects", [])}
        added = []
        for track in self.snapshot(confirmed_only=True):
            if track["associated_object_id"] or track["id"] in existing_ids:
                continue
            rooms = result.get("rooms", [])
            if not rooms:
                continue
            room = min(rooms, key=lambda item: math.dist(
                track["center"][:2], item.get("centroid_xy_m", track["center"][:2])
            ))
            room.setdefault("objects", []).append({
                "id": track["id"], "label": track["label"],
                "center_xyz_m": track["center"], "size_xyz_m": track["size"],
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                "probability": track["score"], "support_count": track["support_count"],
                "source": "online_owlv2_rgbd_fusion", "room_assignment": "nearest_centroid",
            })
            added.append(track["id"])
        return result, added
