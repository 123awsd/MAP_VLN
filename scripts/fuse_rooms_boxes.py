#!/usr/bin/env python3
"""Assign Boxer objects to OccuSG rooms and emit a compact scene graph."""

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


ROOM_CUES = {
    "bedroom": {"bed", "nightstand", "dresser", "wardrobe"},
    "living_room": {"sofa", "couch", "television", "tv", "coffee table"},
    "dining_room": {"dining table", "table", "chair"},
    "kitchen": {"refrigerator", "oven", "microwave", "sink", "cabinet"},
    "bathroom": {"toilet", "bathtub", "shower", "sink"},
    "office": {"desk", "computer", "monitor", "office chair"},
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
                scores[room_type] += probability
    if not scores:
        return "unknown", 0.0
    room_type, score = scores.most_common(1)[0]
    return room_type, round(float(score), 4)


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

    unassigned = []
    with args.boxes.open(newline="", encoding="utf-8") as handle:
        for index, row in enumerate(csv.DictReader(handle)):
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
        room["semantic_type"], room["semantic_score"] = classify_room(room["objects"])

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
            "room_count": len(rooms),
            "object_count": sum(len(room["objects"]) for room in rooms),
            "adjacency_edge_count": len(edges),
            "nearest_fallback_object_ids": unassigned,
        },
        "rooms": rooms,
        "room_adjacency_edges": [{"source": a, "target": b} for a, b in edges],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(graph, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(graph["summary"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
