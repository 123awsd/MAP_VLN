#!/usr/bin/env python3
"""Assign Boxer objects to OccuSG rooms and emit a compact scene graph."""

import argparse
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


def merge_regions_split_by_beds(rooms, box_rows, resolution=0.05, contact_m=0.25):
    """Merge only non-corridor regions split by the same large bed footprint."""
    all_points = [point for room in rooms for point in room["polygon_xy_m"]]
    lower = np.floor(np.min(np.asarray(all_points), axis=0) / resolution).astype(int) - 30
    upper = np.ceil(np.max(np.asarray(all_points), axis=0) / resolution).astype(int) + 30
    width, height = (upper - lower + 1).astype(int)

    def pixels(points):
        return np.asarray([
            np.rint(np.asarray(point) / resolution).astype(int) - lower
            for point in points
        ], dtype=np.int32)

    def room_mask(room):
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [pixels(room["polygon_xy_m"])], 1)
        return mask

    merges = []
    for row_index, row in enumerate(box_rows):
        if str(row.get("name", "")).strip().lower() != "bed":
            continue
        size = np.asarray([float(row["scale_x"]), float(row["scale_y"])])
        if float(np.prod(size)) < 1.0:
            continue
        center = np.asarray([float(row["tx_world_object"]), float(row["ty_world_object"])])
        w, x, y, z = [float(row[key]) for key in (
            "qw_world_object", "qx_world_object", "qy_world_object", "qz_world_object"
        )]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        half = 0.55 * size
        footprint = [
            (center + rotation @ (half * np.asarray([sx, sy]))).tolist()
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        ]
        bed_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(bed_mask, [pixels(footprint)], 1)
        radius = max(1, int(round(contact_m / resolution)))
        kernel = np.ones((2 * radius + 1, 2 * radius + 1), dtype=np.uint8)
        candidates = []
        masks = {}
        for room in rooms:
            if is_transition_geometry(room):
                continue
            mask = room_mask(room)
            contact = int(np.count_nonzero(cv2.dilate(mask, kernel) & bed_mask))
            if contact >= 20:
                candidates.append(room)
                masks[room["id"]] = mask
        if len(candidates) < 2:
            continue

        merged_ids = {room["id"] for room in candidates}
        union = bed_mask.copy()
        for room in candidates:
            union |= masks[room["id"]]
        contours, _ = cv2.findContours(union, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=cv2.contourArea)
        contour = cv2.approxPolyDP(contour, 1.0, True).reshape(-1, 2)
        polygon = [((point + lower) * resolution).astype(float).tolist() for point in contour]
        moments = cv2.moments(union)
        centroid_pixel = np.asarray([
            moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
        ])
        centroid = ((centroid_pixel + lower) * resolution).astype(float).tolist()
        merged = {
            "id": min(merged_ids),
            "centroid_xy_m": centroid,
            "area_m2": float(np.count_nonzero(union) * resolution * resolution),
            "polygon_xy_m": polygon,
            "adjacent_room_ids": sorted({
                value for room in candidates for value in room["adjacent_room_ids"]
                if value not in merged_ids
            }),
            "objects": [],
            "merged_from_region_ids": sorted(merged_ids),
            "merge_reason": f"large_bed_boxer_{row_index}",
        }
        rooms = [room for room in rooms if room["id"] not in merged_ids] + [merged]
        replacement = {room_id: merged["id"] for room_id in merged_ids}
        for room in rooms:
            room["adjacent_room_ids"] = sorted({
                replacement.get(value, value) for value in room["adjacent_room_ids"]
                if replacement.get(value, value) != room["id"]
            })
        merges.append({"object_id": f"boxer_{row_index}", "region_ids": sorted(merged_ids), "result_id": merged["id"]})
    return sorted(rooms, key=lambda room: room["id"]), merges


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("regions", type=Path)
    parser.add_argument("boxes", type=Path)
    parser.add_argument("output", type=Path)
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

    with args.boxes.open(newline="", encoding="utf-8") as handle:
        box_rows = list(csv.DictReader(handle))
    rooms, furniture_merges = merge_regions_split_by_beds(rooms, box_rows)

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

    for room in rooms:
        room["semantic_type"], room["semantic_score"], room["space_role"] = classify_region(room)
    attach_small_fragments(rooms)

    valid_ids = {room["id"] for room in rooms}
    edges = sorted({
        tuple(sorted((room["id"], adjacent)))
        for room in rooms for adjacent in room["adjacent_room_ids"]
        if adjacent in valid_ids and adjacent != room["id"]
    })
    graph = {
        "format": "pre_map_vln.scene_graph.v1",
        "coordinate_frame": "falcon_world_z_up",
        "sources": {"regions": str(args.regions), "boxes": str(args.boxes)},
        "summary": {
            "source_region_count": int(region_data["region_count"]),
            "region_count": len(rooms),
            "room_count": sum(room["space_role"] == "room" for room in rooms),
            "object_count": sum(len(room["objects"]) for room in rooms),
            "adjacency_edge_count": len(edges),
            "corridor_count": sum(room["space_role"] == "transition_space" for room in rooms),
            "fragment_count": sum(room["space_role"] == "room_fragment" for room in rooms),
            "furniture_region_merge_count": len(furniture_merges),
            "nearest_fallback_object_ids": unassigned,
        },
        "rooms": rooms,
        "room_adjacency_edges": [{"source": a, "target": b} for a, b in edges],
        "furniture_region_merges": furniture_merges,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(graph, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(graph["summary"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
