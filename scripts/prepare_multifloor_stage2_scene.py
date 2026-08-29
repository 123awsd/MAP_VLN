#!/usr/bin/env python3
"""Flatten per-floor semantic graphs into the stage-two multi-floor contract."""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from stage2.io_utils import atomic_json  # noqa: E402


def global_room_id(floor: int, room_id) -> str:
    return f"L{floor}_R{room_id}"


def global_object_id(floor: int, object_id) -> str:
    return f"L{floor}_{object_id}"


def flatten_scene(source: dict, wall_grid_dir: Path, transitions: dict | None = None) -> dict:
    rooms = []
    floor_records = []
    seen_rooms, seen_objects = set(), set()
    for floor_graph in source.get("floors", []):
        floor = int(floor_graph["floor_id"])
        metadata_path = wall_grid_dir / f"L{floor}_wall_grid.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        camera_band = metadata["single_floor_policy"]["camera_height_band_m"]
        local_room_ids = {room["id"] for room in floor_graph.get("rooms", [])}
        floor_room_ids = []
        for source_room in floor_graph.get("rooms", []):
            room = copy.deepcopy(source_room)
            local_id = room["id"]
            room["source_room_id"] = local_id
            room["id"] = global_room_id(floor, local_id)
            room["floor_id"] = floor
            room["floor_z_m"] = float(metadata["floor_z_m"])
            room["camera_height_band_m"] = [float(value) for value in camera_band]
            room["adjacent_room_ids"] = [
                global_room_id(floor, value) for value in room.get("adjacent_room_ids", [])
                if value in local_room_ids
            ]
            if "parent_room_id" in room:
                room["parent_room_id"] = global_room_id(floor, room["parent_room_id"])
            for obj in room.get("objects", []):
                local_object_id = obj["id"]
                obj["source_object_id"] = local_object_id
                obj["id"] = global_object_id(floor, local_object_id)
                obj["room_id"] = room["id"]
                obj["floor_id"] = floor
                if obj["id"] in seen_objects:
                    raise ValueError(f'duplicate global object ID: {obj["id"]}')
                seen_objects.add(obj["id"])
            if room["id"] in seen_rooms:
                raise ValueError(f'duplicate global room ID: {room["id"]}')
            seen_rooms.add(room["id"])
            floor_room_ids.append(room["id"])
            rooms.append(room)
        floor_records.append({
            "floor_id": floor, "floor_z_m": float(metadata["floor_z_m"]),
            "camera_height_band_m": [float(value) for value in camera_band],
            "room_ids": floor_room_ids, "wall_grid_metadata": str(metadata_path.resolve()),
        })

    stairwells = copy.deepcopy(source.get("stairwells", []))
    for stairwell in stairwells:
        stairwell["floor_room_ids"] = {
            str(floor): [global_room_id(int(floor), room_id) for room_id in room_ids]
            for floor, room_ids in stairwell.pop("floor_region_ids", {}).items()
        }
    transition_records = []
    if transitions is not None:
        for item in transitions.get("transitions", []):
            transition_records.append({
                "id": item["id"], "connected_floors": item["connected_floors"],
                "lower_entrance_xyz": item["lower_entrance_xyz"],
                "upper_entrance_xyz": item["upper_entrance_xyz"],
                "centerline_xyz": (
                    item.get("traversals", [{}])[0].get("centerline_xyz", [])
                    if item.get("traversals") else []
                ),
                "confidence": item.get("confidence"),
                "classification": item.get("classification"),
            })
    objects = [obj for room in rooms for obj in room.get("objects", [])]
    edges = sorted({
        tuple(sorted((room["id"], adjacent)))
        for room in rooms for adjacent in room.get("adjacent_room_ids", [])
    })
    return {
        "format": "pre_map_vln.multifloor_stage2_scene_graph.v1",
        "coordinate_frame": "falcon_world_z_up",
        "rooms": rooms, "objects": objects, "floors": floor_records,
        "stairwells": stairwells, "transitions": transition_records,
        "room_adjacency_edges": [{"source": a, "target": b} for a, b in edges],
        "summary": {
            "floor_count": len(floor_records), "room_count": len(rooms),
            "object_count": len(objects), "stairwell_count": len(stairwells),
            "transition_count": len(transition_records),
            "globally_unique_room_ids": len(seen_rooms) == len(rooms),
            "globally_unique_object_ids": len(seen_objects) == len(objects),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_graph", type=Path)
    parser.add_argument("wall_grid_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--transitions", type=Path, default=None)
    args = parser.parse_args()
    source = json.loads(args.source_graph.read_text(encoding="utf-8"))
    transitions = (
        None if args.transitions is None
        else json.loads(args.transitions.read_text(encoding="utf-8"))
    )
    graph = flatten_scene(source, args.wall_grid_dir, transitions)
    atomic_json(args.output, graph)
    print(json.dumps(graph["summary"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
