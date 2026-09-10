#!/usr/bin/env python3
"""Interactive per-map editor for OccuSG room proposals.

The editor changes only semantic room polygons. It never changes the voxel map
used for collision checking and never starts a ROS or flight process.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
import cv2
from matplotlib.path import Path as MplPath
from matplotlib.patches import Polygon
from matplotlib.widgets import Button, PolygonSelector, TextBox


ROLES = ("room", "transition_space")


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def polygon_area(vertices: list[list[float]]) -> float:
    points = np.asarray(vertices, dtype=float)
    return 0.5 * abs(float(np.dot(points[:, 0], np.roll(points[:, 1], -1))
                           - np.dot(points[:, 1], np.roll(points[:, 0], -1))))


def polygon_centroid(vertices: list[list[float]]) -> list[float]:
    points = np.asarray(vertices, dtype=float)
    signed = 0.5 * float(np.dot(points[:, 0], np.roll(points[:, 1], -1))
                         - np.dot(points[:, 1], np.roll(points[:, 0], -1)))
    if abs(signed) < 1e-9:
        return points.mean(axis=0).tolist()
    cross = points[:, 0] * np.roll(points[:, 1], -1) - np.roll(points[:, 0], -1) * points[:, 1]
    cx = float(np.sum((points[:, 0] + np.roll(points[:, 0], -1)) * cross) / (6.0 * signed))
    cy = float(np.sum((points[:, 1] + np.roll(points[:, 1], -1)) * cross) / (6.0 * signed))
    return [cx, cy]


def simplify_proposal(vertices: list[list[float]], resolution: float) -> list[list[float]]:
    """Remove OccuSG's repeated/touching boundary samples for safe manual editing."""
    contour = np.asarray(vertices, dtype=np.float32)
    simplified = cv2.approxPolyDP(contour, max(0.05, 2.0 * resolution), True)
    return [[float(x), float(y)] for x, y in simplified[:, 0, :]]


def segment_distance(a, b, c, d) -> float:
    def point_segment(point, first, second):
        point, first, second = map(lambda item: np.asarray(item, dtype=float), (point, first, second))
        delta = second - first
        denominator = float(np.dot(delta, delta))
        ratio = 0.0 if denominator <= 1e-12 else float(np.clip(np.dot(point - first, delta) / denominator, 0, 1))
        return float(np.linalg.norm(point - (first + ratio * delta)))
    return min(point_segment(a, c, d), point_segment(b, c, d),
               point_segment(c, a, b), point_segment(d, a, b))


def polygon_gap(first: list[list[float]], second: list[list[float]]) -> float:
    if MplPath(first).contains_point(second[0]) or MplPath(second).contains_point(first[0]):
        return 0.0
    return min(
        segment_distance(first[index], first[(index + 1) % len(first)],
                         second[other], second[(other + 1) % len(second)])
        for index in range(len(first)) for other in range(len(second))
    )


class RoomEditor:
    def __init__(self, args):
        self.args = args
        self.grid = np.load(args.grid)
        self.metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
        self.origin = np.asarray(self.metadata["origin_xy_m"], dtype=float)
        self.resolution = float(self.metadata["resolution_m"])
        self.height, self.width = self.grid.shape
        voxel = json.loads(args.voxel_metadata.read_text(encoding="utf-8"))
        voxel_origin = np.asarray(voxel["origin_xyz_m"], dtype=float)
        voxel_upper = voxel_origin + np.asarray(voxel["shape_xyz"], dtype=float) * float(voxel["resolution_m"])
        planning_start = json.loads(args.planning_start.read_text(encoding="utf-8"))["start_xyz_yaw"]
        start_z = float(planning_start[2])
        self.default_z = (max(float(voxel_origin[2]), start_z - 0.5),
                          min(float(voxel_upper[2]), start_z + 0.5))
        self.rooms = self.load_rooms()
        self.selected: set[str] = set()
        self.selector = None
        self.selector_mode = None
        self.selector_room_id = None
        self.artists = []
        self.updating_fields = False
        self.saved = False

        self.fig, self.ax = plt.subplots(figsize=(14, 9))
        plt.subplots_adjust(bottom=0.29, right=0.98)
        extent = [self.origin[0], self.origin[0] + self.width * self.resolution,
                  self.origin[1], self.origin[1] + self.height * self.resolution]
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[self.grid == -1] = (100, 100, 100)
        image[self.grid == 0] = (245, 245, 245)
        image[self.grid == 100] = (25, 25, 25)
        self.ax.imshow(image, origin="lower", extent=extent, interpolation="nearest")
        self.ax.set_aspect("equal")
        source_role = self.metadata.get("map_role", "room_structure")
        self.ax.set_title(f"{args.run_id}: {source_role} + OccuSG manual room review")
        self.ax.set_xlabel("world x [m]")
        self.ax.set_ylabel("world y [m]")
        self.draw_clusters()
        self.create_controls()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.redraw()

    def load_rooms(self):
        if self.args.draft.is_file():
            document = json.loads(self.args.draft.read_text(encoding="utf-8"))
            self.discarded_region_ids = document.get("discarded_region_ids", [])
            return document["rooms"]
        raw = json.loads(self.args.regions.read_text(encoding="utf-8"))
        eligible = [region for region in raw["regions"]
                    if float(region.get("area_m2", 0.0)) >= self.args.min_room_area]
        self.discarded_region_ids = [region["id"] for region in raw["regions"] if region not in eligible]
        id_map = {str(region["id"]): f"L{self.args.floor_id}_R{index:02d}"
                  for index, region in enumerate(eligible, 1)}
        rooms = []
        for index, region in enumerate(eligible, 1):
            rooms.append({
                "id": id_map[str(region["id"])],
                "name": f"room_{index}",
                "aliases": [],
                "floor_id": self.args.floor_id,
                "space_role": "room",
                "polygon_xy_m": simplify_proposal(region["polygon_xy_m"], self.resolution),
                "z_min_m": self.default_z[0],
                "z_max_m": self.default_z[1],
                "adjacent_room_ids": [id_map[str(value)] for value in region.get("adjacent_ids", [])
                                      if str(value) in id_map],
                "source_region_ids": [region["id"]],
            })
        return rooms

    def draw_clusters(self):
        if not self.args.clusters.is_file():
            return
        clusters = json.loads(self.args.clusters.read_text(encoding="utf-8"))
        selected = [item for item in clusters if int(item.get("observations", 0)) >= self.args.min_observations]
        if not selected:
            return
        self.ax.scatter([item["world_x_m"] for item in selected],
                        [item["world_y_m"] for item in selected],
                        marker="x", s=28, c="#ef4444", zorder=4,
                        label=f"Box clusters (observations >= {self.args.min_observations})")
        self.ax.legend(loc="upper right")

    def create_controls(self):
        def button(rect, label, callback):
            widget = Button(self.fig.add_axes(rect), label)
            widget.on_clicked(callback)
            return widget
        self.buttons = [
            button([0.04, 0.19, 0.075, 0.045], "New", self.new_polygon),
            button([0.12, 0.19, 0.075, 0.045], "Edit", self.edit_polygon),
            button([0.20, 0.19, 0.075, 0.045], "Delete", self.delete_rooms),
            button([0.28, 0.19, 0.075, 0.045], "Merge", self.merge_rooms),
            button([0.36, 0.19, 0.075, 0.045], "Role", self.cycle_role),
            button([0.44, 0.19, 0.09, 0.045], "Auto Adj", self.auto_adjacency),
            button([0.54, 0.19, 0.11, 0.045], "Save Draft", self.save_and_close),
        ]
        self.name_box = TextBox(self.fig.add_axes([0.08, 0.12, 0.20, 0.045]), "Name ")
        self.zmin_box = TextBox(self.fig.add_axes([0.36, 0.12, 0.10, 0.045]), "flight z min ")
        self.zmax_box = TextBox(self.fig.add_axes([0.53, 0.12, 0.10, 0.045]), "flight z max ")
        self.adj_box = TextBox(self.fig.add_axes([0.74, 0.12, 0.20, 0.045]), "Adjacent IDs ")
        for widget in (self.name_box, self.zmin_box, self.zmax_box, self.adj_box):
            widget.on_submit(self.apply_fields)
        self.status = self.fig.text(
            0.04, 0.045,
            "Click: select | Shift-click: multi-select | New/Edit: click vertices, Enter to finish | "
            "Split: delete then draw two | Save Draft closes editor",
            fontsize=9,
        )

    def room_by_id(self, room_id):
        return next(room for room in self.rooms if room["id"] == room_id)

    def set_status(self, value):
        self.status.set_text(value)
        self.fig.canvas.draw_idle()

    def redraw(self):
        for artist in self.artists:
            artist.remove()
        self.artists = []
        for room in self.rooms:
            selected = room["id"] in self.selected
            base = "#38bdf8" if room["space_role"] == "transition_space" else "#22c55e"
            patch = Polygon(room["polygon_xy_m"], closed=True,
                            facecolor=base, edgecolor="#facc15" if selected else base,
                            linewidth=3.0 if selected else 1.5, alpha=0.24, zorder=3)
            self.ax.add_patch(patch)
            center = polygon_centroid(room["polygon_xy_m"])
            label = self.ax.text(center[0], center[1], f'{room["id"]}\n{room["name"]}',
                                 ha="center", va="center", fontsize=8, zorder=5,
                                 color="#fde047" if selected else "#111827")
            self.artists.extend((patch, label))
        self.refresh_fields()
        self.fig.canvas.draw_idle()

    def refresh_fields(self):
        self.updating_fields = True
        if len(self.selected) == 1:
            room = self.room_by_id(next(iter(self.selected)))
            self.name_box.set_val(room["name"])
            self.zmin_box.set_val(f'{room["z_min_m"]:.2f}')
            self.zmax_box.set_val(f'{room["z_max_m"]:.2f}')
            self.adj_box.set_val(",".join(room.get("adjacent_room_ids", [])))
        else:
            for widget in (self.name_box, self.zmin_box, self.zmax_box, self.adj_box):
                widget.set_val("")
        self.updating_fields = False

    def apply_fields(self, _value=None):
        if self.updating_fields or len(self.selected) != 1:
            return
        room = self.room_by_id(next(iter(self.selected)))
        try:
            name = self.name_box.text.strip()
            z_min = float(self.zmin_box.text)
            z_max = float(self.zmax_box.text)
            if not name or not math.isfinite(z_min) or not math.isfinite(z_max) or z_min >= z_max:
                raise ValueError
        except ValueError:
            self.set_status("Invalid name or z range; require a non-empty name and z_min < z_max.")
            return
        room["name"] = name
        room["z_min_m"] = z_min
        room["z_max_m"] = z_max
        room["adjacent_room_ids"] = [item.strip() for item in self.adj_box.text.split(",") if item.strip()]
        self.redraw()

    def on_click(self, event):
        if self.selector is not None or event.inaxes != self.ax or event.xdata is None:
            return
        matches = [room["id"] for room in self.rooms
                   if MplPath(room["polygon_xy_m"]).contains_point((event.xdata, event.ydata))]
        if not matches:
            if event.key != "shift":
                self.selected.clear()
        else:
            room_id = matches[-1]
            if event.key == "shift":
                self.selected.symmetric_difference_update({room_id})
            else:
                self.selected = {room_id}
        self.redraw()

    def next_room_id(self):
        used = {room["id"] for room in self.rooms}
        index = 1
        while f"L{self.args.floor_id}_R{index:02d}" in used:
            index += 1
        return f"L{self.args.floor_id}_R{index:02d}"

    def start_selector(self, mode, room_id=None):
        if self.selector is not None:
            self.selector.disconnect_events()
        self.selector_mode = mode
        self.selector_room_id = room_id
        self.selector = PolygonSelector(self.ax, self.finish_selector, useblit=True)
        if mode == "edit":
            self.selector.verts = self.room_by_id(room_id)["polygon_xy_m"]
        self.set_status(f"{mode.title()} polygon: click/drag vertices and press Enter to finish.")

    def finish_selector(self, vertices):
        points = [[float(x), float(y)] for x, y in vertices]
        if len(points) < 3 or polygon_area(points) < 0.25:
            self.set_status("Polygon rejected: at least 3 vertices and 0.25 m^2 are required.")
        elif self.selector_mode == "new":
            room_id = self.next_room_id()
            self.rooms.append({
                "id": room_id, "name": room_id.lower(), "aliases": [],
                "floor_id": self.args.floor_id, "space_role": "room",
                "polygon_xy_m": points, "z_min_m": self.default_z[0],
                "z_max_m": self.default_z[1], "adjacent_room_ids": [],
                "source_region_ids": [],
            })
            self.selected = {room_id}
        else:
            self.room_by_id(self.selector_room_id)["polygon_xy_m"] = points
        self.selector.disconnect_events()
        self.selector = None
        self.selector_mode = None
        self.redraw()

    def new_polygon(self, _event):
        self.start_selector("new")

    def edit_polygon(self, _event):
        if len(self.selected) != 1:
            self.set_status("Select exactly one room before Edit.")
            return
        self.start_selector("edit", next(iter(self.selected)))

    def delete_rooms(self, _event):
        if not self.selected:
            self.set_status("Select one or more rooms before Delete.")
            return
        removed = set(self.selected)
        self.rooms = [room for room in self.rooms if room["id"] not in removed]
        for room in self.rooms:
            room["adjacent_room_ids"] = [value for value in room.get("adjacent_room_ids", []) if value not in removed]
        self.selected.clear()
        self.redraw()

    def merge_rooms(self, _event):
        if len(self.selected) < 2:
            self.set_status("Shift-click at least two adjacent rooms before Merge.")
            return
        mask = np.zeros(self.grid.shape, dtype=np.uint8)
        selected_rooms = [room for room in self.rooms if room["id"] in self.selected]
        for room in selected_rooms:
            pixels = np.rint((np.asarray(room["polygon_xy_m"]) - self.origin) / self.resolution).astype(np.int32)
            cv2.fillPoly(mask, [pixels], 1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 1:
            self.set_status("Selected rooms are disconnected; only adjacent rooms can be merged.")
            return
        contour = cv2.approxPolyDP(contours[0], max(1.0, 0.5 / self.resolution), True)[:, 0, :]
        polygon = (self.origin + contour.astype(float) * self.resolution).tolist()
        primary = selected_rooms[0]
        primary["polygon_xy_m"] = polygon
        primary["source_region_ids"] = sorted({value for room in selected_rooms for value in room.get("source_region_ids", [])})
        primary["adjacent_room_ids"] = sorted({value for room in selected_rooms for value in room.get("adjacent_room_ids", [])
                                                  if value not in self.selected})
        removed = self.selected - {primary["id"]}
        self.rooms = [room for room in self.rooms if room["id"] not in removed]
        for room in self.rooms:
            room["adjacent_room_ids"] = sorted({primary["id"] if value in removed else value
                                                  for value in room.get("adjacent_room_ids", [])
                                                  if value != room["id"]})
        self.selected = {primary["id"]}
        self.redraw()

    def cycle_role(self, _event):
        if len(self.selected) != 1:
            self.set_status("Select exactly one room before changing its role.")
            return
        room = self.room_by_id(next(iter(self.selected)))
        room["space_role"] = ROLES[(ROLES.index(room["space_role"]) + 1) % len(ROLES)]
        self.redraw()

    def auto_adjacency(self, _event):
        for room in self.rooms:
            room["adjacent_room_ids"] = []
        for index, first in enumerate(self.rooms):
            for second in self.rooms[index + 1:]:
                if polygon_gap(first["polygon_xy_m"], second["polygon_xy_m"]) <= self.args.adjacency_gap_m:
                    first["adjacent_room_ids"].append(second["id"])
                    second["adjacent_room_ids"].append(first["id"])
        self.redraw()
        self.set_status(f"Adjacency regenerated with maximum polygon gap {self.args.adjacency_gap_m:.2f} m.")

    def save_and_close(self, _event):
        self.apply_fields()
        document = {
            "format": "pre_map_vln.real_room_review_draft.v1",
            "run_id": self.args.run_id,
            "floor_id": self.args.floor_id,
            "min_object_observations": self.args.min_observations,
            "min_room_area_m2": self.args.min_room_area,
            "discarded_region_ids": self.discarded_region_ids,
            "source_regions": str(self.args.regions.resolve()),
            "rooms": self.rooms,
        }
        atomic_json(self.args.draft, document)
        self.fig.savefig(self.args.preview, dpi=180, bbox_inches="tight")
        self.saved = True
        plt.close(self.fig)

    def run(self):
        plt.show()
        if not self.saved:
            raise SystemExit("Editor closed without Save Draft; no approval was created.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--grid", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--regions", type=Path, required=True)
    parser.add_argument("--clusters", type=Path, required=True)
    parser.add_argument("--voxel-metadata", type=Path, required=True)
    parser.add_argument("--planning-start", type=Path, required=True)
    parser.add_argument("--draft", type=Path, required=True)
    parser.add_argument("--preview", type=Path, required=True)
    parser.add_argument("--min-observations", type=int, required=True)
    parser.add_argument("--min-room-area", type=float, default=5.0)
    parser.add_argument("--floor-id", type=int, default=1)
    parser.add_argument("--adjacency-gap-m", type=float, default=0.45)
    args = parser.parse_args()
    RoomEditor(args).run()


if __name__ == "__main__":
    main()
