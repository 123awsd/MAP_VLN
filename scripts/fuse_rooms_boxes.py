#!/usr/bin/env python3
"""Assign Boxer objects to OccuSG rooms and emit a compact scene graph."""

import argparse
import copy
import csv
import json
import math
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


ROOM_CUES = {
    "bedroom": {"bed", "nightstand", "dresser", "wardrobe"},
    "living_room": {"sofa", "couch", "television", "tv", "coffee table"},
    "dining_room": {"dining table", "table", "chair"},
    "kitchen": {"refrigerator", "oven", "microwave", "sink", "cabinet"},
    "bathroom": {"toilet", "bathtub", "shower", "sink"},
    "office": {"desk", "computer", "monitor", "office chair"},
}

# A bed, toilet or oven is a much stronger room cue than a TV, chair or generic
# cabinet. Equal voting previously mislabeled a bedroom containing a TV as a
# living room.
CUE_WEIGHTS = {
    "bed": 2.0, "nightstand": 1.5, "dresser": 1.2, "wardrobe": 1.2,
    "sofa": 1.6, "couch": 1.6, "television": 0.7, "tv": 0.7,
    "refrigerator": 1.8, "oven": 1.8, "microwave": 1.5, "sink": 1.3,
    "toilet": 2.0, "bathtub": 2.0, "shower": 2.0,
    "desk": 1.5, "computer": 1.5, "monitor": 1.2,
    "cabinet": 0.35, "chair": 0.25, "table": 0.4, "dining table": 1.4,
}


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
    return inside


def filter_navigation_exclusions(rooms, grid_metadata):
    """Remove regions centered on semantic no-go structures such as stairs."""
    exclusion_regions = (
        (grid_metadata or {}).get("single_floor_policy", {})
        .get("excluded_navigation_regions", [])
    )
    excluded_ids = {
        room["id"] for room in rooms
        if any(
            point_in_polygon(room["centroid_xy_m"], exclusion["polygon_xy_m"])
            for exclusion in exclusion_regions
        )
    }
    if not excluded_ids:
        return rooms, []
    filtered = [room for room in rooms if room["id"] not in excluded_ids]
    for room in filtered:
        room["adjacent_room_ids"] = [
            room_id for room_id in room["adjacent_room_ids"] if room_id not in excluded_ids
        ]
    return filtered, sorted(excluded_ids)


def filter_navigation_exclusion_islands(rooms, grid_metadata, maximum_gap_m=0.80):
    """Remove small empty islands left beside a blocked stair assembly."""
    exclusion_regions = (
        (grid_metadata or {}).get("single_floor_policy", {})
        .get("excluded_navigation_regions", [])
    )
    excluded_ids = set()
    for room in rooms:
        if room.get("objects") or room.get("adjacent_room_ids") or float(room["area_m2"]) > 2.0:
            continue
        for exclusion in exclusion_regions:
            gap = min(
                math.dist(room_vertex, exclusion_vertex)
                for room_vertex in room["polygon_xy_m"]
                for exclusion_vertex in exclusion["polygon_xy_m"]
            )
            if gap <= maximum_gap_m:
                excluded_ids.add(room["id"])
                break
    return [room for room in rooms if room["id"] not in excluded_ids], sorted(excluded_ids)


def classify_room(objects):
    scores = Counter()
    for obj in objects:
        label = obj["label"].lower()
        probability = obj["probability"]
        for room_type, cues in ROOM_CUES.items():
            if label in cues:
                scores[room_type] += probability * CUE_WEIGHTS.get(label, 1.0)
    if not scores:
        return "unknown", 0.0
    room_type, score = scores.most_common(1)[0]
    return room_type, round(float(score), 4)


def classify_region(room):
    """Separate narrow transition spaces from semantic destination rooms."""
    area = float(room["area_m2"])
    degree = len(room["adjacent_room_ids"])
    if area <= 2.0 and degree >= 2:
        return "corridor", 1.0, "transition_space"
    room_type, score = classify_room(room["objects"])
    return room_type, score, "room"


def is_transition_geometry(room):
    return float(room["area_m2"]) <= 2.0 and len(room["adjacent_room_ids"]) >= 2


def attach_small_fragments(rooms, minimum_room_area_m2=1.0):
    """Keep tiny polygons as geometry, but do not present them as extra rooms."""
    major_rooms = [
        room for room in rooms
        if room["space_role"] == "room" and float(room["area_m2"]) >= minimum_room_area_m2
    ]
    if not major_rooms:
        return
    for room in rooms:
        if room["space_role"] != "room" or float(room["area_m2"]) >= minimum_room_area_m2:
            continue
        parent = min(
            major_rooms,
            key=lambda candidate: math.dist(room["centroid_xy_m"], candidate["centroid_xy_m"]),
        )
        room["space_role"] = "room_fragment"
        room["parent_room_id"] = parent["id"]
        room["local_semantic_type"] = room["semantic_type"]
    for parent in major_rooms:
        attached_objects = list(parent.get("objects", []))
        attached_objects.extend(
            obj for room in rooms if room.get("parent_room_id") == parent["id"]
            for obj in room.get("objects", [])
        )
        if attached_objects:
            parent["semantic_type"], parent["semantic_score"] = classify_room(attached_objects)
    for room in rooms:
        if room.get("space_role") == "room_fragment":
            parent = next(item for item in major_rooms if item["id"] == room["parent_room_id"])
            room["semantic_type"] = parent["semantic_type"]
            room["semantic_score"] = parent["semantic_score"]


def merge_polygon_components(components, resolution=0.05, bridge_width_m=0.20):
    """Return one display contour, bridging only components already assigned to one room."""
    if len(components) == 1:
        return copy.deepcopy(components[0])
    all_points = np.asarray([point for polygon in components for point in polygon], dtype=np.float64)
    lower = np.floor(np.min(all_points, axis=0) / resolution).astype(int) - 4
    upper = np.ceil(np.max(all_points, axis=0) / resolution).astype(int) + 4
    width, height = (upper - lower + 1).astype(int)

    def pixels(points):
        return np.asarray([
            np.rint(np.asarray(point) / resolution).astype(int) - lower
            for point in points
        ], dtype=np.int32)

    pixel_components = [pixels(polygon) for polygon in components]
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, pixel_components, 1)
    connected_vertices = pixel_components[0].reshape(-1, 2)
    thickness = max(1, int(round(bridge_width_m / resolution)))
    for polygon in pixel_components[1:]:
        vertices = polygon.reshape(-1, 2)
        distances = np.sum(
            (connected_vertices[:, None, :] - vertices[None, :, :]) ** 2, axis=2
        )
        source_index, target_index = np.unravel_index(np.argmin(distances), distances.shape)
        cv2.line(
            mask,
            tuple(int(value) for value in connected_vertices[source_index]),
            tuple(int(value) for value in vertices[target_index]),
            1,
            thickness=thickness,
        )
        connected_vertices = np.concatenate([connected_vertices, vertices], axis=0)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    contour = cv2.approxPolyDP(contour, 1.0, True).reshape(-1, 2)
    return [((point + lower) * resolution).astype(float).tolist() for point in contour]


def canonicalize_rooms(rooms):
    """Aggregate fragments into semantic rooms while retaining raw geometry provenance."""
    source_regions = []
    for room in rooms:
        source = copy.deepcopy(room)
        source["object_ids"] = [obj["id"] for obj in source.pop("objects", [])]
        source_regions.append(source)

    parent_by_id = {
        room["id"]: room.get("parent_room_id", room["id"])
        if room.get("space_role") == "room_fragment" else room["id"]
        for room in rooms
    }
    groups = {}
    for room in rooms:
        groups.setdefault(parent_by_id[room["id"]], []).append(room)

    canonical = []
    for canonical_id, members in sorted(groups.items()):
        parent = next((room for room in members if room["id"] == canonical_id), None)
        if parent is None:
            raise ValueError(f"fragment group {canonical_id} has no parent room")
        result = copy.deepcopy(parent)
        components = [copy.deepcopy(room["polygon_xy_m"]) for room in members]
        result["polygon_components_xy_m"] = components
        result["polygon_xy_m"] = merge_polygon_components(components)
        result["merged_from_region_ids"] = sorted(room["id"] for room in members)
        result["area_m2"] = float(sum(float(room["area_m2"]) for room in members))
        if result["area_m2"] > 0:
            result["centroid_xy_m"] = (
                sum(
                    np.asarray(room["centroid_xy_m"], dtype=np.float64) * float(room["area_m2"])
                    for room in members
                ) / result["area_m2"]
            ).astype(float).tolist()
        result["objects"] = []
        for room in members:
            for obj in room.get("objects", []):
                obj = copy.deepcopy(obj)
                obj["source_region_id"] = room["id"]
                obj["canonical_room_id"] = canonical_id
                result["objects"].append(obj)
        result.pop("parent_room_id", None)
        result.pop("local_semantic_type", None)
        result["space_role"] = parent["space_role"]
        adjacent = {
            parent_by_id.get(adjacent_id, adjacent_id)
            for room in members for adjacent_id in room.get("adjacent_room_ids", [])
        }
        result["adjacent_room_ids"] = sorted(
            room_id for room_id in adjacent if room_id != canonical_id
        )
        canonical.append(result)

    valid_ids = {room["id"] for room in canonical}
    for room in canonical:
        room["adjacent_room_ids"] = sorted({
            room_id for room_id in room["adjacent_room_ids"] if room_id in valid_ids
        })
    return canonical, source_regions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("regions", type=Path)
    parser.add_argument("boxes", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--grid-meta", type=Path, default=None,
        help="occupancy metadata containing single-floor navigation exclusions",
    )
    args = parser.parse_args()

    region_data = json.loads(args.regions.read_text(encoding="utf-8"))
    rooms = []
    for source in region_data["regions"]:
        rooms.append({
            "id": int(source["id"]),
            "centroid_xy_m": source["centroid_xy_m"],
            "area_m2": source["area_m2"],
            "polygon_xy_m": source["polygon_xy_m"],
            "adjacent_room_ids": source["adjacent_ids"],
            "objects": [],
        })
    if not rooms:
        raise ValueError("OccuSG returned no rooms")
    grid_metadata = None if args.grid_meta is None else json.loads(args.grid_meta.read_text())
    rooms, excluded_navigation_region_ids = filter_navigation_exclusions(rooms, grid_metadata)
    if not rooms:
        raise ValueError("all OccuSG regions were removed by navigation exclusions")

    with args.boxes.open(newline="", encoding="utf-8") as handle:
        box_rows = list(csv.DictReader(handle))
    unassigned = []
    for index, row in enumerate(box_rows):
        center = [float(row["tx_world_object"]), float(row["ty_world_object"])]
        candidates = [room for room in rooms if point_in_polygon(center, room["polygon_xy_m"])]
        assignment = "inside_polygon"
        if not candidates:
            # Boxes on a wall or just outside an incomplete explored contour are
            # retained, but the fallback is explicit in the output.
            candidates = [min(rooms, key=lambda room: math.dist(center, room["centroid_xy_m"]))]
            assignment = "nearest_centroid_fallback"
        room = min(candidates, key=lambda item: math.dist(center, item["centroid_xy_m"]))
        obj = {
            "id": f"boxer_{index}",
            "label": row["name"],
            "center_xyz_m": [
                float(row["tx_world_object"]),
                float(row["ty_world_object"]),
                float(row["tz_world_object"]),
            ],
            "size_xyz_m": [float(row["scale_x"]), float(row["scale_y"]), float(row["scale_z"])],
            "orientation_wxyz": [
                float(row["qw_world_object"]), float(row["qx_world_object"]),
                float(row["qy_world_object"]), float(row["qz_world_object"]),
            ],
            "probability": float(row["prob"]),
            "room_assignment": assignment,
        }
        room["objects"].append(obj)
        if assignment != "inside_polygon":
            unassigned.append(obj["id"])

    rooms, excluded_navigation_island_ids = filter_navigation_exclusion_islands(
        rooms, grid_metadata
    )
    excluded_navigation_region_ids = sorted(set(
        excluded_navigation_region_ids + excluded_navigation_island_ids
    ))

    for room in rooms:
        room["semantic_type"], room["semantic_score"], room["space_role"] = classify_region(room)
    attach_small_fragments(rooms)
    source_fragment_count = sum(room["space_role"] == "room_fragment" for room in rooms)
    rooms, geometric_regions = canonicalize_rooms(rooms)

    valid_ids = {room["id"] for room in rooms}
    edges = sorted({
        tuple(sorted((room["id"], adjacent)))
        for room in rooms for adjacent in room["adjacent_room_ids"]
        if adjacent in valid_ids and adjacent != room["id"]
    })
    graph = {
        "format": "pre_map_vln.scene_graph.v1",
        "coordinate_frame": "falcon_world_z_up",
        "sources": {
            "regions": str(args.regions), "boxes": str(args.boxes),
            "grid_metadata": None if args.grid_meta is None else str(args.grid_meta),
        },
        "summary": {
            "source_region_count": int(region_data["region_count"]),
            "excluded_navigation_region_count": len(excluded_navigation_region_ids),
            "excluded_navigation_region_ids": excluded_navigation_region_ids,
            "region_count": len(rooms),
            "room_count": sum(room["space_role"] == "room" for room in rooms),
            "object_count": sum(len(room["objects"]) for room in rooms),
            "adjacency_edge_count": len(edges),
            "corridor_count": sum(room["space_role"] == "transition_space" for room in rooms),
            "fragment_count": sum(room["space_role"] == "room_fragment" for room in rooms),
            "source_fragment_count": source_fragment_count,
            "nearest_fallback_object_ids": unassigned,
        },
        "rooms": rooms,
        "geometric_regions": geometric_regions,
        "room_adjacency_edges": [{"source": a, "target": b} for a, b in edges],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(graph, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(graph["summary"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
