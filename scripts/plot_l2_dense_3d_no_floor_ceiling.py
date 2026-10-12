#!/usr/bin/env python3
"""Render a flat L2 RGB-D overview with only measured wall points kept in 3D."""
from __future__ import annotations

import argparse
import heapq
import json
import re
import struct
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
import numpy as np
import cv2
from PIL import Image
from scipy.ndimage import distance_transform_edt

from plot_dense_rgbd_floor_pointcloud import depth_points
from plot_partial_exploration_pointcloud import voxel_first


def truth_wall_support(mesh_path: Path, config_path: Path, shape: tuple[int, int],
                       origin: np.ndarray, resolution: float) -> np.ndarray:
    """Vertical HM3D mesh walls seen in several L2 horizontal sections."""
    blob = mesh_path.read_bytes()
    json_size = struct.unpack_from("<I", blob, 12)[0]
    gltf = json.loads(blob[20:20 + json_size])
    buffer_start = 20 + json_size + 8
    views, accessors = gltf["bufferViews"], gltf["accessors"]

    def accessor(index: int) -> np.ndarray:
        info = accessors[index]
        view = views[info["bufferView"]]
        dtype = {5126: "<f4", 5123: "<u2", 5125: "<u4"}[info["componentType"]]
        width = {"SCALAR": 1, "VEC3": 3}[info["type"]]
        item_size = np.dtype(dtype).itemsize
        offset = buffer_start + view["byteOffset"] + info.get("byteOffset", 0)
        stride = view.get("byteStride", item_size * width)
        return np.ndarray((info["count"], width), dtype=dtype, buffer=blob,
                          offset=offset, strides=(stride, item_size)).copy()

    triangles = []
    for mesh in gltf["meshes"]:
        for primitive in mesh["primitives"]:
            vertices = accessor(primitive["attributes"]["POSITION"])
            indices = accessor(primitive["indices"]).reshape(-1, 3)
            triangles.append(vertices[indices])
    init_x, init_y, init_z = json.loads(config_path.read_text())[
        "initial_agent_habitat_xyz"]
    floor_height = init_y + 1.6
    masks = []
    for above_floor in (0.85, 1.20, 1.55):
        plane = floor_height + above_floor
        mask = np.zeros(shape, dtype=np.uint8)
        for mesh_triangles in triangles:
            heights = mesh_triangles[:, :, 2]
            crossing = mesh_triangles[(heights.min(axis=1) < plane)
                                      & (heights.max(axis=1) > plane)]
            for triangle in crossing:
                intersections = []
                for a, b in ((0, 1), (1, 2), (2, 0)):
                    va, vb = triangle[a], triangle[b]
                    if (va[2] - plane) * (vb[2] - plane) < 0:
                        intersections.append(va + (plane - va[2]) /
                                             (vb[2] - va[2]) * (vb - va))
                if len(intersections) != 2:
                    continue
                xy = np.asarray([[p[1] + init_z, -(p[0] - init_x)]
                                 for p in intersections])
                cells = np.rint((xy - origin) / resolution).astype(np.int32)
                cv2.line(mask, tuple(cells[0]), tuple(cells[1]), 1, 1)
        masks.append(mask)
    neighbor = np.ones((3, 3), np.uint8)
    return ((masks[1] > 0)
            & (cv2.dilate(masks[0], neighbor) > 0)
            & (cv2.dilate(masks[2], neighbor) > 0))


def wall_bounded_room_labels(scene: dict, wall_grid: np.ndarray,
                             origin: np.ndarray, resolution: float) -> np.ndarray:
    """Grow rooms through free space, then color only the building below walls."""
    shape = wall_grid.shape
    free = wall_grid == 0
    labels = np.zeros(shape, dtype=np.int16)
    for room in scene["rooms"]:
        polygon = np.rint((np.asarray(room["polygon_xy_m"]) - origin)
                           / resolution).astype(np.int32)
        seed = np.zeros(shape, dtype=np.uint8)
        cv2.fillPoly(seed, [polygon], 1)
        labels[(seed > 0) & free] = int(room["id"])
    distance = np.full(shape, np.inf, np.float32)
    heap = []
    for row, col in np.argwhere(labels > 0):
        distance[row, col] = 0
        heap.append((0.0, int(row), int(col), int(labels[row, col])))
    heapq.heapify(heap)
    height, width = shape
    while heap:
        cost, row, col, rid = heapq.heappop(heap)
        if cost > distance[row, col] + 1e-5 or labels[row, col] != rid:
            continue
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = row + dr, col + dc
            if not (0 <= rr < height and 0 <= cc < width and free[rr, cc]):
                continue
            candidate = cost + 1
            if candidate < distance[rr, cc]:
                distance[rr, cc] = candidate
                labels[rr, cc] = rid
                heapq.heappush(heap, (candidate, rr, cc, rid))
    # Assign wall-footprint cells to the closest room after free-space growth.
    # This closes the broad white strip below a wall while never extending to
    # the exterior unknown. Enclosed unknown components are small map holes.
    exterior_unknown = wall_grid == -1
    count, components, stats, _ = cv2.connectedComponentsWithStats(
        exterior_unknown.astype(np.uint8))
    exterior_id = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]) if count > 1 else 0
    distance_from_free = distance_transform_edt(~free) * resolution
    enclosed_holes = exterior_unknown & (components != exterior_id)
    inside_building = free | ((wall_grid == 100) & (distance_from_free <= .18))
    inside_building |= enclosed_holes & (distance_from_free <= .18)
    nearest = distance_transform_edt(labels == 0, return_distances=False,
                                     return_indices=True)
    holes_and_walls = inside_building & (labels == 0)
    labels[holes_and_walls] = labels[tuple(nearest)][holes_and_walls]
    # Some room interiors are marked unknown where depth never reached the
    # floor. Fill only inside that room's own footprint (convex for small
    # enclosed rooms), never by expanding its color into a neighbor.
    for room in scene["rooms"]:
        rid = int(room["id"])
        polygon = np.rint((np.asarray(room["polygon_xy_m"]) - origin)
                           / resolution).astype(np.int32)
        if room["semantic_type"] in ("bedroom", "bathroom"):
            polygon = cv2.convexHull(polygon).reshape(-1, 2)
        mask = np.zeros(shape, dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        labels[(mask > 0) & (labels == 0)] = rid
    # Close narrow blank seams in the colored floor support. The two larger
    # enclosed voids remain open as genuine shaft/column spaces.
    colored = (labels > 0).astype(np.uint8)
    closed = cv2.morphologyEx(
        colored, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)))
    gap_distance, nearest = distance_transform_edt(
        labels == 0, return_indices=True)
    short_seams = (closed > 0) & (labels == 0) & (gap_distance < 22)
    labels[short_seams] = labels[tuple(nearest)][short_seams]
    return labels


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("grid_metadata", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--maximum-depth", type=float, default=5.0)
    parser.add_argument("--voxel", type=float, default=0.03)
    parser.add_argument("--slice-center-above-floor", type=float, default=1.20)
    parser.add_argument("--slice-height", type=float, default=0.50)
    parser.add_argument("--overview-floor-clearance", type=float, default=0.80)
    parser.add_argument("--overview-ceiling-clearance", type=float, default=0.95)
    parser.add_argument("--overview-wall-proximity", type=float, default=0.18,
                        help="Keep overview points only this many metres from wall-grid cells")
    parser.add_argument("--room-scene-graph", type=Path, default=None,
                        help="Optional room graph: fill every room with semantic floor points")
    parser.add_argument("--floor-footprint-html", type=Path, default=None,
                        help="Boxer view containing the L2 building contour")
    parser.add_argument("--floor-footprint-inset", type=float, default=0.22,
                        help="Pull the visualized floor in from the loose source contour")
    parser.add_argument("--strict-room-bounds", action="store_true",
                        help="Color only each room's own segmented footprint")
    parser.add_argument("--wall-bounded-rooms", action="store_true",
                        help="Fill known free space from room seeds without crossing walls")
    parser.add_argument("--strict-room-margin", type=float, default=0.20,
                        help="Fill small gaps between each room and its wall")
    parser.add_argument("--neutral-unassigned-floor", action="store_true",
                        help="Show still-unassigned free floor in neutral gray")
    parser.add_argument("--repair-wall-gaps", action="store_true",
                        help="Add inferred points only at short supported wall gaps")
    parser.add_argument("--truth-mesh", type=Path, default=None,
                        help="Optional HM3D mesh for validating longer missing wall runs")
    parser.add_argument("--truth-config", type=Path, default=None)
    parser.add_argument("--outer-gap-grid-window", type=int, nargs=4,
                        metavar=("X0", "Y0", "X1", "Y1"), default=None,
                        help="Wall-grid window of a confirmed exterior wall gap")
    parser.add_argument("--room-point-spacing", type=float, default=0.025)
    parser.add_argument("--room-fill-margin", type=float, default=0.25,
                        help="Extend room floor points into adjacent known free cells")
    parser.add_argument("--transition-fill-margin", type=float, default=0.55,
                        help="Allow narrow hall and stair regions to fill nearby free cells")
    parser.add_argument("--tight-crop", action="store_true",
                        help="Crop the exported PNG around the rendered building")
    parser.add_argument("--storey-height", type=float, default=2.80)
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=26.0)
    parser.add_argument("--z-exaggeration", type=float, default=1.55)
    args = parser.parse_args()

    metadata = json.loads(args.grid_metadata.read_text())
    floor_z = float(metadata["floor_z_m"])
    slice_center_z = floor_z + args.slice_center_above_floor
    z_min = slice_center_z - args.slice_height / 2.0
    z_max = slice_center_z + args.slice_height / 2.0
    origin = np.asarray(metadata["origin_xy_m"], dtype=float)
    resolution = float(metadata["resolution_m"])
    x_min = float(origin[0])
    x_max = x_min + int(metadata["width"]) * resolution
    y_min = float(origin[1])
    y_max = y_min + int(metadata["height"]) * resolution

    pieces = []
    frames = sorted(args.episode.glob("frame_*.npz"))
    overview_min = floor_z + args.overview_floor_clearance
    overview_max = floor_z + args.storey_height - args.overview_ceiling_clearance
    for frame in frames:
        points = depth_points(frame, args.pixel_stride, args.maximum_depth)
        keep = (
            (points[:, 2] >= overview_min) & (points[:, 2] <= overview_max)
            & (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
            & (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
        )
        if np.any(keep):
            pieces.append(points[keep])
    if not pieces:
        raise RuntimeError("No L2 points remain after floor/ceiling filtering")
    raw_points = np.concatenate(pieces)
    points, _ = voxel_first(raw_points, args.voxel)

    # Use the measured wall grid only as a selector. No wall geometry is
    # synthesized: selected points are still the original RGB-D samples.
    wall_grid = np.load(args.grid_metadata.with_name(args.grid_metadata.stem + ".npy"))
    gx = np.floor((points[:, 0] - x_min) / resolution).astype(int)
    gy = np.floor((points[:, 1] - y_min) / resolution).astype(int)
    valid_cell = ((gx >= 0) & (gx < wall_grid.shape[1])
                  & (gy >= 0) & (gy < wall_grid.shape[0]))
    wall_selector = np.zeros(len(points), dtype=bool)
    wall_selector[valid_cell] = wall_grid[gy[valid_cell], gx[valid_cell]] == 100
    wall_selector &= (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    wall_points = points[wall_selector]
    gap_points = np.empty((0, 3), dtype=np.float32)
    gap_cells = 0
    outer_gap_cells = 0
    if args.repair_wall_gaps:
        observed = np.zeros(wall_grid.shape, dtype=np.uint8)
        observed[gy[wall_selector], gx[wall_selector]] = 1
        # Closing the measured 2-D support itself repairs only short gaps;
        # the wall grid confirms locations, but never creates new wall runs.
        closed_observed = cv2.morphologyEx(
            observed, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
        already_seen = cv2.dilate(observed, np.ones((3, 3), np.uint8)) > 0
        missing = (closed_observed > 0) & (wall_grid == 100) & ~already_seen
        if args.truth_mesh is not None:
            if args.truth_config is None:
                raise ValueError("--truth-config is required with --truth-mesh")
            truth = truth_wall_support(args.truth_mesh, args.truth_config,
                                       wall_grid.shape, origin, resolution)
            near_grid_wall = distance_transform_edt(wall_grid != 100) * resolution <= .15
            truth_missing = truth & near_grid_wall & ~already_seen
            missing |= truth_missing
        if args.outer_gap_grid_window is not None:
            x0, y0, x1, y1 = args.outer_gap_grid_window
            exterior_unknown = wall_grid == -1
            _, components, stats, _ = cv2.connectedComponentsWithStats(
                exterior_unknown.astype(np.uint8))
            exterior = components == 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            exterior_neighbor = cv2.dilate(exterior.astype(np.uint8),
                                           np.ones((3, 3), np.uint8)) > 0
            closed_structure = cv2.morphologyEx(
                (wall_grid == 100).astype(np.uint8), cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)))
            outer_bridge = (closed_structure > 0) & (wall_grid != 100)
            outer_bridge &= exterior_neighbor
            window = np.zeros(wall_grid.shape, dtype=bool)
            window[y0:y1, x0:x1] = True
            outer_bridge &= window
            outer_gap_cells = int(outer_bridge.sum())
            missing |= outer_bridge
        row, col = np.where(missing)
        gap_cells = len(row)
        heights = np.linspace(z_min + .015, z_max - .015, 8)
        gap_points = np.column_stack((
            np.repeat(x_min + (col + .5) * resolution, len(heights)),
            np.repeat(y_min + (row + .5) * resolution, len(heights)),
            np.tile(heights, gap_cells))).astype(np.float32)

    # The old overview projected every non-wall point into a false-looking
    # floor sheet. Limit this flat context to a narrow band beside walls.
    distance_to_wall = distance_transform_edt(wall_grid != 100) * resolution
    overview_selector = np.zeros(len(points), dtype=bool)
    overview_selector[valid_cell] = (
        distance_to_wall[gy[valid_cell], gx[valid_cell]]
        <= args.overview_wall_proximity
    )
    overview_points = points[overview_selector]

    # The overview is a conventional top projection. Only points classified as
    # wall returns keep their measured height within the requested 0.3 m slice.
    overview_points = overview_points[np.argsort(overview_points[:, 2])]
    figure = plt.figure(figsize=(10, 9), facecolor="white")
    axis = figure.add_subplot(111, projection="3d", computed_zorder=False)
    base_z = z_min - 0.035
    room_point_count = 0
    neutral_point_count = 0
    if args.room_scene_graph is not None:
        scene = json.loads(args.room_scene_graph.read_text())
        # Separate soft tones by room ID so neighboring rooms are distinguishable.
        palette = {
            1: "#b7cfe8", 2: "#d6c6e8", 3: "#e3bedb", 4: "#accfd2",
            5: "#d3ddaa", 6: "#eac9b7", 7: "#b8dbca", 8: "#f1d39d",
            9: "#d3dbe5", 10: "#afc4dd", 11: "#c8bfda", 12: "#c9dfb7",
        }
        spacing = args.room_point_spacing
        xx = np.arange(x_min + spacing / 2, x_max, spacing)
        yy = np.arange(y_min + spacing / 2, y_max, spacing)
        floor_xy = np.stack(np.meshgrid(xx, yy), axis=-1).reshape(-1, 2)
        room_id = np.zeros(len(floor_xy), dtype=np.int16)
        for room in scene["rooms"]:
            polygon = np.asarray(room["polygon_xy_m"])
            room_id[MplPath(polygon).contains_points(floor_xy)] = int(room["id"])
        room_grid = room_id.reshape(len(yy), len(xx))
        source_room_grid = room_grid.copy()
        room_seed_mask = room_grid > 0
        distance, nearest = distance_transform_edt(
            room_grid == 0, sampling=spacing, return_indices=True)
        nearest_room = room_grid[tuple(nearest)]
        if args.floor_footprint_html is not None:
            match = re.search(r"const DATA=(.*?), \$=id",
                              args.floor_footprint_html.read_text())
            if match is None:
                raise ValueError("Cannot find DATA in floor footprint HTML")
            contours = json.loads(match.group(1))["contours"]
            footprint = np.zeros(room_grid.shape, dtype=np.uint8)
            for contour in contours:
                polygon = np.asarray(contour)
                pixel = np.rint((polygon - [xx[0], yy[0]]) / spacing).astype(np.int32)
                cv2.fillPoly(footprint, [pixel], 1)
            close_radius = max(1, int(round(0.38 / spacing)))
            close_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * close_radius + 1, 2 * close_radius + 1))
            footprint = cv2.morphologyEx(footprint, cv2.MORPH_CLOSE, close_kernel)
            radius = max(0, int(round(args.floor_footprint_inset / spacing)))
            if radius:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                    (2 * radius + 1, 2 * radius + 1))
                footprint = cv2.erode(footprint, kernel)
            # The approximate exterior contour can cut into a predicted room.
            # Always retain the source room interior while completing the gaps.
            footprint[room_seed_mask] = 1
            extend = (room_grid == 0) & (footprint > 0)
        else:
            gx_floor = np.floor((floor_xy[:, 0] - x_min) / resolution).astype(int)
            gy_floor = np.floor((floor_xy[:, 1] - y_min) / resolution).astype(int)
            known_free = wall_grid[gy_floor, gx_floor] == 0
            transition_ids = [int(room["id"]) for room in scene["rooms"]
                              if room["semantic_type"] in ("entry_hall", "stairwell")]
            margin = np.where(np.isin(nearest_room, transition_ids),
                              args.transition_fill_margin, args.room_fill_margin)
            extend = (room_grid == 0) & (distance <= margin)
            extend &= known_free.reshape(room_grid.shape)
        room_grid[extend] = nearest_room[extend]
        if args.floor_footprint_html is not None:
            room_grid[footprint == 0] = 0
        if args.strict_room_bounds:
            # Avoid nearest-room extrapolation across real walls. Fill only the
            # source regions and their tiny raster gaps at their own boundary.
            room_grid = source_room_grid.copy()
            for room in scene["rooms"]:
                rid = int(room["id"])
                polygon = np.asarray(room["polygon_xy_m"])
                pixel = np.rint((polygon - [xx[0], yy[0]]) / spacing).astype(np.int32)
                mask = np.zeros(room_grid.shape, dtype=np.uint8)
                if room["semantic_type"] in ("bedroom", "bathroom"):
                    pixel = cv2.convexHull(pixel).reshape(-1, 2)
                cv2.fillPoly(mask, [pixel], 1)
                close_radius = max(1, int(round(0.14 / spacing)))
                close_kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (2 * close_radius + 1, 2 * close_radius + 1))
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
                allowed = (source_room_grid == 0) | (source_room_grid == rid)
                if args.floor_footprint_html is not None:
                    allowed &= (footprint > 0) | (source_room_grid == rid)
                room_grid[(mask > 0) & allowed] = rid
            gx_floor = np.floor((floor_xy[:, 0] - x_min) / resolution).astype(int)
            gy_floor = np.floor((floor_xy[:, 1] - y_min) / resolution).astype(int)
            not_wall = wall_grid[gy_floor, gx_floor].reshape(room_grid.shape) != 100
            small_gap = (room_grid == 0) & (distance <= args.strict_room_margin)
            small_gap &= not_wall
            if args.floor_footprint_html is not None:
                small_gap &= footprint > 0
            room_grid[small_gap] = nearest_room[small_gap]
        if args.wall_bounded_rooms:
            wall_labels = wall_bounded_room_labels(scene, wall_grid, origin, resolution)
            gx_floor = np.floor((floor_xy[:, 0] - x_min) / resolution).astype(int)
            gy_floor = np.floor((floor_xy[:, 1] - y_min) / resolution).astype(int)
            room_grid = wall_labels[gy_floor, gx_floor].reshape(room_grid.shape)
        if args.neutral_unassigned_floor and args.floor_footprint_html is not None:
            gx_floor = np.floor((floor_xy[:, 0] - x_min) / resolution).astype(int)
            gy_floor = np.floor((floor_xy[:, 1] - y_min) / resolution).astype(int)
            known_free = wall_grid[gy_floor, gx_floor].reshape(room_grid.shape) == 0
            neutral_mask = (room_grid == 0) & (footprint > 0) & known_free
            neutral_xy = floor_xy[neutral_mask.ravel()]
            neutral_point_count = len(neutral_xy)
            axis.scatter(neutral_xy[:, 0], neutral_xy[:, 1],
                         np.full(neutral_point_count, base_z - .019),
                         s=1.6, c="#e5eaec", alpha=.55, linewidths=0,
                         depthshade=False, rasterized=True, zorder=0)
        room_id = room_grid.ravel()
        for room in scene["rooms"]:
            selected = floor_xy[room_id == int(room["id"])]
            room_point_count += len(selected)
            axis.scatter(selected[:, 0], selected[:, 1],
                         np.full(len(selected), base_z - .018),
                         s=2.0, c=palette.get(int(room["id"]), "#d9e0e4"),
                         alpha=.78, linewidths=0, depthshade=False,
                         rasterized=True, zorder=1)
    axis.scatter(overview_points[:, 0], overview_points[:, 1],
                 np.full(len(overview_points), base_z),
                 s=.25, c="#527d94", alpha=.50, linewidths=0,
                 depthshade=False, rasterized=True, zorder=2)
    axis.scatter(wall_points[:, 0], wall_points[:, 1], wall_points[:, 2],
                 s=.42, c="#164e70", alpha=.90, linewidths=0,
                 depthshade=True, rasterized=True, zorder=3)
    if gap_cells:
        axis.scatter(gap_points[:, 0], gap_points[:, 1], gap_points[:, 2],
                     s=.34, c="#245a76", alpha=.78, linewidths=0,
                     depthshade=True, rasterized=True, zorder=3)
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    axis.set_zlim(base_z - .03, z_max)
    axis.set_box_aspect((x_max - x_min, y_max - y_min,
                         (z_max - z_min) * args.z_exaggeration))
    axis.set_proj_type("ortho")
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.set_axis_off()
    figure.subplots_adjust(left=0, right=1, bottom=0, top=1)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=240, bbox_inches="tight", pad_inches=.02,
                   facecolor="white")
    plt.close(figure)
    if args.tight_crop:
        image = Image.open(args.output).convert("RGB")
        data = np.asarray(image)
        occupied = np.any(data < 246, axis=2)
        if np.any(occupied):
            rows, cols = np.where(occupied)
            padding = 85
            box = (max(0, int(cols.min()) - padding),
                   max(0, int(rows.min()) - padding),
                   min(image.width, int(cols.max()) + padding + 1),
                   min(image.height, int(rows.max()) + padding + 1))
            image.crop(box).save(args.output)

    report = {
        "format": "pre_map_vln.l2_flat_overview_3d_walls.v1",
        "floor": int(metadata.get("floor", 2)),
        "source_episode": str(args.episode.resolve()),
        "source_grid_metadata": str(args.grid_metadata.resolve()),
        "recorded_frames": len(frames),
        "filter": {
            "floor_z_m": floor_z,
            "kept_z_band_m": [z_min, z_max],
            "slice_center_above_floor_m": args.slice_center_above_floor,
            "slice_height_m": args.slice_height,
            "overview_source_z_band_m": [overview_min, overview_max],
            "overview_wall_proximity_m": args.overview_wall_proximity,
        },
        "raw_filtered_points": int(len(raw_points)),
        "voxelized_points": int(len(points)),
        "overview_rendered_points": int(len(overview_points)),
        "semantic_floor_points": room_point_count,
        "neutral_unassigned_floor_points": neutral_point_count,
        "room_fill_margin_m": args.room_fill_margin if args.room_scene_graph else None,
        "transition_fill_margin_m": args.transition_fill_margin if args.room_scene_graph else None,
        "tight_crop": args.tight_crop,
        "room_scene_graph": str(args.room_scene_graph.resolve()) if args.room_scene_graph else None,
        "floor_footprint_html": str(args.floor_footprint_html.resolve()) if args.floor_footprint_html else None,
        "floor_footprint_inset_m": args.floor_footprint_inset if args.floor_footprint_html else None,
        "strict_room_bounds": args.strict_room_bounds,
        "wall_bounded_rooms": args.wall_bounded_rooms,
        "strict_room_margin_m": args.strict_room_margin if args.strict_room_bounds else None,
        "measured_wall_points_in_3d": int(len(wall_points)),
        "inferred_wall_gap_cells": gap_cells,
        "inferred_outer_bridge_cells": outer_gap_cells,
        "inferred_wall_gap_points": int(len(gap_points)),
        "outer_gap_grid_window": args.outer_gap_grid_window,
        "truth_wall_mesh": str(args.truth_mesh.resolve()) if args.truth_mesh else None,
        "non_wall_rendering": "only overview points near wall-grid cells projected to one horizontal plane",
        "wall_rendering": "RGB-D points selected by wall grid retain measured Z",
        "voxel_m": args.voxel,
        "pixel_stride": args.pixel_stride,
        "uses_rgb": False,
        "geometry_source": "recorded RGB-D depth",
        "view": {"azimuth": args.azimuth, "elevation": args.elevation,
                 "projection": "orthographic", "z_exaggeration": args.z_exaggeration},
        "png": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
