#!/usr/bin/env python3
"""Run FALCON against a progressively revealed real-map voxel snapshot.

The complete map is used only by this offline sensor simulator.  FALCON sees
ordinary RGB-D frames and odometry through the existing atomic-file bridge.
No ROS control or flight hardware is accessed by this program.
"""

import argparse
import heapq
import json
import math
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
from matplotlib.path import Path as PolygonPath
from scipy.ndimage import binary_dilation
from scipy.ndimage import distance_transform_edt
from scipy.ndimage import label


R_BODY_CAMERA = np.asarray(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64
)


def atomic_bytes(path: Path, value: bytes) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_bytes(value)
    os.replace(temp, path)


def atomic_json(path: Path, value: dict) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")
    os.replace(temp, path)


def matrix_to_xyzw(matrix: np.ndarray) -> list:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = (
            0.25 * s,
            (matrix[2, 1] - matrix[1, 2]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
            (matrix[1, 0] - matrix[0, 1]) / s,
        )
    else:
        i = int(np.argmax(np.diag(matrix)))
        if i == 0:
            s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w, x, y, z = (matrix[2, 1] - matrix[1, 2]) / s, 0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w, x, y, z = (matrix[0, 2] - matrix[2, 0]) / s, (matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w, x, y, z = (matrix[1, 0] - matrix[0, 1]) / s, (matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s
    value = np.asarray([x, y, z, w], dtype=np.float64)
    value /= np.linalg.norm(value)
    return value.tolist()


def load_environment(
    snapshot: Path,
    scene_graph_path: Path,
    excluded_names: set,
    boundary_path: Optional[Path] = None,
):
    metadata = json.loads((snapshot / "metadata.json").read_text(encoding="utf-8"))
    arrays = np.load(snapshot / "voxel_map.npz")
    occupancy = arrays["raw_occupancy_zyx"]
    esdf = arrays["esdf_zyx_m"]
    resolution = float(metadata["resolution_m"])
    origin = np.asarray(metadata["origin_xyz_m"], dtype=np.float64)

    scene = json.loads(scene_graph_path.read_text(encoding="utf-8"))
    included = [room for room in scene["rooms"] if room["name"] not in excluded_names]
    excluded = [room for room in scene["rooms"] if room["name"] in excluded_names]
    polygons = {room["name"]: PolygonPath(np.asarray(room["polygon_xy_m"])) for room in included}

    ny, nx = occupancy.shape[1:]
    xs = origin[0] + (np.arange(nx) + 0.5) * resolution
    ys = origin[1] + (np.arange(ny) + 0.5) * resolution
    xx, yy = np.meshgrid(xs, ys)
    samples = np.column_stack((xx.ravel(), yy.ravel()))
    if boundary_path is not None:
        boundary = json.loads(boundary_path.read_text(encoding="utf-8"))
        polygon_xy = np.asarray(boundary["allowed_polygon_xy_m"], dtype=np.float64)
        if polygon_xy.ndim != 2 or polygon_xy.shape[0] < 3 or polygon_xy.shape[1] != 2:
            raise RuntimeError(f"invalid exploration boundary: {boundary_path}")
        boundary_polygon = PolygonPath(polygon_xy)
        allowed = boundary_polygon.contains_points(samples, radius=1e-7).reshape(ny, nx)
        for item in boundary.get("excluded_polygons", []):
            vertices = item.get("polygon_xy_m", item) if isinstance(item, dict) else item
            blocked_xy = np.asarray(vertices, dtype=np.float64)
            if blocked_xy.ndim != 2 or blocked_xy.shape[0] < 3 or blocked_xy.shape[1] != 2:
                raise RuntimeError(f"invalid excluded polygon in: {boundary_path}")
            blocked = PolygonPath(blocked_xy).contains_points(
                samples, radius=1e-7
            ).reshape(ny, nx)
            allowed &= ~blocked
        polygons = {"approved_exploration_boundary": boundary_polygon}
        included = []
        excluded = []
    else:
        allowed = np.zeros(len(samples), dtype=bool)
        for polygon in polygons.values():
            allowed |= polygon.contains_points(samples, radius=1e-7)
        allowed = allowed.reshape(ny, nx)
        # Hand-approved room polygons intentionally stop at wall/door boundaries
        # and can leave a sub-metre annotation gap between adjacent rooms.  Join
        # those doorway gaps without opening the excluded rooms themselves.
        allowed = binary_dilation(allowed, iterations=max(1, int(math.ceil(1.0 / resolution))))
        for room in excluded:
            blocked = PolygonPath(np.asarray(room["polygon_xy_m"])).contains_points(
                samples, radius=1e-7
            ).reshape(ny, nx)
            allowed &= ~blocked

    # Flight permission must not alter sensor geometry: preserve real
    # obstacles outside the polygon and never synthesize geofence returns.
    zz, yyi, xxi = np.nonzero(occupancy == 100)
    real_points = origin + (np.column_stack((xxi, yyi, zz)) + 0.5) * resolution
    real_points = real_points[
        (real_points[:, 2] >= 0.0)
        & (real_points[:, 2] <= 2.4)
    ]
    points = real_points
    return metadata, occupancy, esdf, origin, resolution, allowed, polygons, included, points


def render_depth(
    occupancy: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    allowed: np.ndarray,
    position: np.ndarray,
    yaw: float,
) -> np.ndarray:
    """Ray-cast the archived occupied voxels at FALCON's 4-pixel stride."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_body = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    rotation_camera = rotation_body @ R_BODY_CAMERA
    uu, vv = np.meshgrid(np.arange(0, 640, 4), np.arange(0, 480, 4))
    ray_camera = np.column_stack((
        (uu.ravel() - 320.0) / 320.0,
        (vv.ravel() - 240.0) / 320.0,
        np.ones(uu.size),
    ))
    ray_world = ray_camera @ rotation_camera.T
    low_flat = np.zeros(uu.size, dtype=np.float32)
    unresolved = np.ones(uu.size, dtype=bool)
    nz, ny, nx = occupancy.shape
    # Half-voxel depth steps avoid stepping over a thin 0.2 m wall. Depth is
    # optical-Z, matching the image convention expected by voxel_mapping.
    for depth in np.arange(0.15, 5.001, min(0.1, resolution * 0.5)):
        if not unresolved.any():
            break
        ids = np.flatnonzero(unresolved)
        world = position + ray_world[ids] * depth
        ix = np.floor((world[:, 0] - origin[0]) / resolution).astype(np.int32)
        iy = np.floor((world[:, 1] - origin[1]) / resolution).astype(np.int32)
        iz = np.floor((world[:, 2] - origin[2]) / resolution).astype(np.int32)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (iz >= 0) & (iz < nz)
        hit = ~inside
        if inside.any():
            valid_ids = np.flatnonzero(inside)
            vx, vy, vz = ix[valid_ids], iy[valid_ids], iz[valid_ids]
            hit[valid_ids] = (
                (occupancy[vz, vy, vx] == 100)
                | (world[valid_ids, 2] < 0.0)
                | (world[valid_ids, 2] > 2.4)
            )
        hit_ids = ids[hit]
        low_flat[hit_ids] = depth
        unresolved[hit_ids] = False
    low = low_flat.reshape(120, 160)
    return np.repeat(np.repeat(low, 4, axis=0), 4, axis=1)


def render_mid360(
    occupancy: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    allowed: np.ndarray,
    position: np.ndarray,
    sensed_free: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Ray-cast a compact MID-360-like scan and return world-frame hits."""
    azimuth = np.deg2rad(np.arange(0.0, 360.0, 2.0))
    elevation = np.deg2rad(np.arange(-7.0, 52.1, 4.0))
    aa, ee = np.meshgrid(azimuth, elevation)
    cos_e = np.cos(ee.ravel())
    rays = np.column_stack((
        cos_e * np.cos(aa.ravel()),
        cos_e * np.sin(aa.ravel()),
        np.sin(ee.ravel()),
    ))
    hits = np.empty((len(rays), 3), dtype=np.float32)
    resolved = np.zeros(len(rays), dtype=bool)
    nz, ny, nx = occupancy.shape
    for distance in np.arange(0.15, 8.001, min(0.1, resolution * 0.5)):
        ids = np.flatnonzero(~resolved)
        if not len(ids):
            break
        world = position + rays[ids] * distance
        ix = np.floor((world[:, 0] - origin[0]) / resolution).astype(np.int32)
        iy = np.floor((world[:, 1] - origin[1]) / resolution).astype(np.int32)
        iz = np.floor((world[:, 2] - origin[2]) / resolution).astype(np.int32)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (iz >= 0) & (iz < nz)
        blocked = ~inside
        if inside.any():
            local = np.flatnonzero(inside)
            vx, vy, vz = ix[local], iy[local], iz[local]
            blocked[local] = (
                (occupancy[vz, vy, vx] == 100)
                | (world[local, 2] < 0.0)
                | (world[local, 2] > 2.4)
            )
            if sensed_free is not None:
                free_local = local[~blocked[local]]
                sensed_free[iz[free_local], iy[free_local], ix[free_local]] = True
        hit_ids = ids[blocked]
        hits[hit_ids] = world[blocked].astype(np.float32)
        resolved[hit_ids] = True
    # No-return rays still establish observed free space. A range beyond the
    # configured 8 m maximum tells the radial ray mapper to clip traversal at
    # 8 m without creating a fictitious occupied endpoint.
    hits[~resolved] = (position + rays[~resolved] * 8.2).astype(np.float32)
    return np.ascontiguousarray(hits, dtype="<f4")


def nearest_free_cell(cell, walkable):
    cy, cx = cell
    if 0 <= cy < walkable.shape[0] and 0 <= cx < walkable.shape[1] and walkable[cy, cx]:
        return (cy, cx)
    for radius in range(1, 20):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if abs(dx) != radius and abs(dy) != radius:
                    continue
                y, x = cy + dy, cx + dx
                if 0 <= y < walkable.shape[0] and 0 <= x < walkable.shape[1] and walkable[y, x]:
                    return (y, x)
    return None


def astar_grid(start, goal, walkable, edge_allowed=None):
    """Return a 2-D free-cell path for the offline coverage fallback."""
    start = nearest_free_cell(start, walkable)
    goal = nearest_free_cell(goal, walkable)
    if start is None or goal is None:
        return []
    frontier = [(0.0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0.0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        cy, cx = current
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nxt = (cy + dy, cx + dx)
            if not (0 <= nxt[0] < walkable.shape[0] and 0 <= nxt[1] < walkable.shape[1]) or not walkable[nxt]:
                continue
            if dy and dx and (not walkable[cy, nxt[1]] or not walkable[nxt[0], cx]):
                continue
            if edge_allowed is not None and not edge_allowed(current, nxt):
                continue
            step = 1.41421356237 if dy and dx else 1.0
            new_cost = cost_so_far[current] + step
            if new_cost < cost_so_far.get(nxt, float("inf")):
                cost_so_far[nxt] = new_cost
                priority = new_cost + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current
    return []


def mark_observed(observed, position, origin, resolution, radius_m, walkable):
    cx = int(math.floor((position[0] - origin[0]) / resolution))
    cy = int(math.floor((position[1] - origin[1]) / resolution))
    radius = max(1, int(math.ceil(radius_m / resolution)))
    y0, y1 = max(0, cy - radius), min(observed.shape[0], cy + radius + 1)
    x0, x1 = max(0, cx - radius), min(observed.shape[1], cx + radius + 1)
    yy, xx = np.ogrid[y0:y1, x0:x1]
    disk = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2
    # Only count cells with an unobstructed line from the current position.
    for ly, lx in np.argwhere(disk & walkable[y0:y1, x0:x1]):
        y, x = y0 + ly, x0 + lx
        steps = max(abs(y - cy), abs(x - cx), 1) * 2 + 1
        ry = np.rint(np.linspace(cy, y, steps)).astype(int)
        rx = np.rint(np.linspace(cx, x, steps)).astype(int)
        if np.all(walkable[ry, rx]):
            observed[y, x] = True


def choose_coverage_fallback(position, observed, walkable, origin, resolution):
    candidates = walkable & ~observed
    if not candidates.any():
        return []
    # Pick a cell furthest from the already observed set, then try the next
    # best cells until a collision-free grid path is found.
    distance = distance_transform_edt(~observed)
    score = np.where(candidates, distance, -1.0)
    flat_ids = np.argsort(score.ravel())[::-1]
    start = (
        int(math.floor((position[1] - origin[1]) / resolution)),
        int(math.floor((position[0] - origin[0]) / resolution)),
    )
    attempts = 0
    for flat_id in flat_ids:
        if score.ravel()[flat_id] <= 0.0:
            break
        goal = np.unravel_index(int(flat_id), score.shape)
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) < 8.0:
            continue
        path = astar_grid(start, goal, walkable)
        if len(path) >= 8:
            return path
        attempts += 1
        if attempts >= 300:
            break
    return []


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument(
        "--boundary", type=Path,
        help="Approved XY exploration polygon JSON; when set, room polygons are ignored",
    )
    parser.add_argument("--bridge-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=480.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument('--command-following', choices=('legacy','bounded'), default='legacy')
    parser.add_argument(
        "--scan-sectors", type=int, default=8,
        help="Number of horizontal sectors used to emulate one MID-360 sweep",
    )
    parser.add_argument(
        "--sensor-model", choices=("camera", "mid360"), default="mid360"
    )
    parser.add_argument("--start", nargs=4, type=float, default=(0.0, 0.0, 1.1, 0.0))
    parser.add_argument("--exclude-room", action="append", default=[])
    parser.add_argument("--finish-hold", type=float, default=2.0)
    parser.add_argument("--min-z", type=float, default=0.5)
    parser.add_argument("--max-z", type=float, default=1.5)
    parser.add_argument("--motion-min-clearance", type=float, default=0.25,
                        help="Independent offline motion gate; cannot be less than 0.25 m")
    parser.add_argument(
        "--boundary-only", action="store_true",
        help="Explore until duration/frontier convergence instead of room visitation",
    )
    parser.add_argument(
        "--coverage-fallback", action="store_true",
        help="Use a safe 2-D coverage waypoint when FALCON is stuck",
    )
    parser.add_argument('--coarse-coverage-stop', type=float, default=0.0,
                        help='Offline evaluation stop at this observed safe XY coverage; 0 disables')
    args = parser.parse_args()
    if not math.isfinite(args.motion_min_clearance) or args.motion_min_clearance < 0.25:
        parser.error("motion-min-clearance must be finite and at least 0.25 m")
    if args.scan_sectors < 1:
        parser.error("--scan-sectors must be positive")
    if args.min_z >= args.max_z:
        parser.error("--min-z must be less than --max-z")
    if not 0 <= args.coarse_coverage_stop <= 1:
        parser.error('--coarse-coverage-stop must be in [0,1]')
    args.bridge_dir.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=False)
    if args.boundary is not None:
        boundary_document = json.loads(args.boundary.read_text(encoding='utf-8'))
        args.boundary = args.output / 'boundary_snapshot.json'
        atomic_json(args.boundary, boundary_document)

    values = load_environment(
        args.snapshot, args.scene_graph, set(args.exclude_room), args.boundary
    )
    metadata, occupancy, esdf, origin, resolution, allowed, polygons, rooms, points = values
    flight_z = 1.1
    z_index = int(math.floor((flight_z - origin[2]) / resolution))
    if not (0 <= z_index < occupancy.shape[0]):
        raise RuntimeError(f"flight height {flight_z} m is outside snapshot")
    walkable = allowed & (occupancy[z_index] == 0) & (esdf[z_index] >= 0.25)
    position = np.asarray(args.start[:3], dtype=np.float64)
    yaw = float(args.start[3])
    if not any(poly.contains_point(position[:2]) for poly in polygons.values()):
        raise RuntimeError(f"start is outside approved exploration region: {position.tolist()}")

    target_rooms = {
        room["name"] for room in rooms
        if room.get("space_role") == "room" and not room["name"].startswith("Corridor")
    }
    if args.boundary_only:
        target_rooms = set()
    visited = set()
    trajectory = []
    observed = np.zeros_like(allowed, dtype=bool)
    sensed_free = np.zeros_like(occupancy, dtype=bool)
    coverage_target = walkable.copy()
    components, _ = label(coverage_target)
    start_cell = np.floor((position-origin)/resolution).astype(int)
    component = components[start_cell[1],start_cell[0]]
    if component:
        coverage_target &= components == component
    height_centres = origin[2]+(np.arange(occupancy.shape[0])+.5)*resolution
    coverage_band = (height_centres >= args.min_z-1e-6) & (height_centres <= args.max_z+1e-6)
    coverage_history = []
    coverage_fraction = 0.0
    stop_reason = 'duration_limit'
    mark_observed(observed, position, origin, resolution, 2.5, walkable)
    fallback_path = []
    fallback_segments = []
    fallback_active = False
    fallback_cooldown_until = 0.0
    last_observed_count = int(observed.sum())
    last_coverage_time = time.monotonic()
    fallback_stuck_window = 15.0
    started, period, sequence = time.monotonic(), 1.0 / args.hz, 0
    all_visited_at = None
    command_diagnostics = {'accepted':0, 'stale':0, 'blocked':0, 'invalid':0}
    rejection_reasons = {}
    rejection_examples = []
    previous_tick = started
    while time.monotonic() - started < args.duration:
        tick = time.monotonic()
        previous_position = position.copy()
        elapsed_tick = max(.001, tick-previous_tick)
        previous_tick = tick
        command_path = args.bridge_dir / "command.json"
        try:
            command = json.loads(command_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, OSError, json.JSONDecodeError):
            command = None
        command_position = None
        motion_blocked = False
        if command is not None:
            command_position = np.asarray(command["position"], dtype=np.float64)

        if fallback_path:
            next_y, next_x = fallback_path.pop(0)
            position = np.asarray(
                [
                    origin[0] + (next_x + 0.5) * resolution,
                    origin[1] + (next_y + 0.5) * resolution,
                    flight_z,
                ],
                dtype=np.float64,
            )
            fallback_active = True
            if not fallback_path:
                fallback_active = False
                fallback_cooldown_until = time.monotonic() + 8.0
        elif command_position is not None:
            rejection_reason = None
            proposed = command_position
            if args.command_following == 'bounded':
                age = time.time() - float(command.get('stamp', 0))
                if not 0 <= age <= 2.0:
                    command_diagnostics['stale'] += 1
                    proposed = position.copy()
                elif not np.isfinite(proposed).all():
                    command_diagnostics['invalid'] += 1
                    proposed = position.copy()
                else:
                    feedforward = np.asarray(command.get('velocity', [0,0,0]), dtype=float)
                    if not np.isfinite(feedforward).all(): feedforward = np.zeros(3)
                    velocity = feedforward + 2.0*(proposed-position)
                    speed = np.linalg.norm(velocity)
                    velocity *= min(1.0, 1.2/max(speed,1e-9))
                    proposed = position + velocity * min(elapsed_tick, .25)
            ix = int(math.floor((proposed[0] - origin[0]) / resolution))
            iy = int(math.floor((proposed[1] - origin[1]) / resolution))
            iz = int(math.floor((proposed[2] - origin[2]) / resolution))
            # The archived snapshot independently gates this offline replay.
            # Unobserved unknown cells remain blocked. Actual simulated lidar
            # traversal can reveal archived unknown as free, just as it does
            # in FALCON; it cannot clear occupied cells or override geofences.
            valid = (
                0 <= ix < allowed.shape[1] and 0 <= iy < allowed.shape[0]
                and 0 <= iz < occupancy.shape[0] and allowed[iy, ix]
                and args.min_z <= proposed[2] <= args.max_z
            )
            if not valid:
                rejection_reason = 'bounds_or_height'
            # Reject old trajectory targets after a coverage transfer. Also
            # check the entire movement, including concave boundary crossings.
            if valid:
                delta = proposed - position
                valid = np.isfinite(proposed).all() and np.linalg.norm(delta) <= 1.5 * period
                if not valid:
                    rejection_reason = 'nonfinite_or_step'
                if valid:
                    samples = position + np.linspace(0, 1, 9)[:, None] * delta
                    cells = np.floor((samples - origin) / resolution).astype(int)
                    for sample, (x, y, z) in zip(samples, cells):
                        if not (0 <= x < allowed.shape[1] and 0 <= y < allowed.shape[0]
                                and 0 <= z < occupancy.shape[0]):
                            rejection_reason = 'sample_outside_grid'
                        elif not allowed[y, x]:
                            rejection_reason = 'xy_boundary'
                        elif occupancy[z, y, x] == 100:
                            rejection_reason = 'occupied'
                        elif occupancy[z, y, x] != 0 and not sensed_free[z, y, x]:
                            rejection_reason = 'unobserved_unknown'
                        elif esdf[z, y, x] < args.motion_min_clearance:
                            rejection_reason = 'clearance'
                        if rejection_reason:
                            valid = False
                            if len(rejection_examples) < 30:
                                rejection_examples.append({'reason':rejection_reason,
                                    'position':position.tolist(), 'sample':sample.tolist(),
                                    'cell_xyz':[int(x),int(y),int(z)],
                                    'command':command_position.tolist()})
                            break
            if valid:
                position = proposed
                yaw = float(command.get("yaw", yaw))
                command_diagnostics['accepted'] += 1
            else:
                motion_blocked = True
                command_diagnostics['blocked'] += 1
                reason = rejection_reason or 'unknown'
                rejection_reasons[reason] = rejection_reasons.get(reason, 0) + 1
                if rejection_reasons[reason] == 1 or rejection_reasons[reason] % 50 == 0:
                    print(f"MOTION_REJECT reason={reason} position={position.round(3).tolist()} target={command_position.round(3).tolist()}", flush=True)

        mark_observed(observed, position, origin, resolution, 2.5, walkable)
        observed_count = int(observed.sum())
        if observed_count >= last_observed_count + 4:
            last_observed_count = observed_count
            last_coverage_time = time.monotonic()

        if (
            args.coverage_fallback
            and not fallback_path
            and time.monotonic() >= fallback_cooldown_until
            and time.monotonic() - last_coverage_time >= fallback_stuck_window
        ):
            fallback_path = choose_coverage_fallback(
                position, observed, walkable, origin, resolution
            )
            if fallback_path:
                fallback_segments.append({
                    "start_xyz": position.tolist(),
                    "cells": len(fallback_path),
                })
                fallback_active = True
                last_coverage_time = time.monotonic()
                print(
                    f"coverage fallback: {len(fallback_path)} cells from "
                    f"{position[:2].round(2).tolist()}", flush=True
                )

        for name, polygon in polygons.items():
            if name in target_rooms and polygon.contains_point(position[:2]):
                visited.add(name)
        trajectory.append([time.monotonic() - started, *position.tolist(), yaw])

        if target_rooms and target_rooms <= visited:
            if all_visited_at is None:
                all_visited_at = time.monotonic()
                print("All requested rooms visited; holding final frames", flush=True)
            elif time.monotonic() - all_visited_at >= args.finish_hold:
                break

        sensor_yaw = yaw
        if args.sensor_model == "mid360":
            lidar = render_mid360(occupancy, origin, resolution, allowed, position, sensed_free)
            ray_seen = sensed_free[coverage_band].any(axis=0) & coverage_target
            coverage_fraction = float(ray_seen.sum()/max(1,coverage_target.sum()))
            coverage_history.append([tick-started, coverage_fraction, int(ray_seen.sum())])
            depth = np.zeros((480, 640), dtype=np.float32)
            atomic_bytes(args.bridge_dir / "lidar_xyz_f32.raw", lidar.tobytes())
        else:
            depth = render_depth(
                occupancy, origin, resolution, allowed, position, sensor_yaw
            )
        depth_u16 = np.clip(np.rint(depth * 1000.0), 0, 65535).astype("<u2")
        shade = np.clip(255.0 - depth * 30.0, 35.0, 220.0).astype(np.uint8)
        rgb = np.repeat(shade[..., None], 3, axis=2)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation_body = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        csy, ssy = math.cos(sensor_yaw), math.sin(sensor_yaw)
        rotation_sensor_body = np.asarray(
            [[csy, -ssy, 0.0], [ssy, csy, 0.0], [0.0, 0.0, 1.0]]
        )
        atomic_bytes(args.bridge_dir / "depth_u16.raw", depth_u16.tobytes())
        atomic_bytes(args.bridge_dir / "rgb_u8.raw", rgb.tobytes())
        atomic_bytes(args.bridge_dir / "semantic_i32.raw", np.zeros((480, 640), dtype="<i4").tobytes())
        atomic_json(args.bridge_dir / "state.json", {
            "sequence": sequence, "width": 640, "height": 480,
            "sensor_model": args.sensor_model,
            "position": position.tolist(),
            "velocity_world": ((position-previous_position)/elapsed_tick).tolist(),
            "preview_motion_blocked": motion_blocked,
            "blocked_trajectory_id": int(command.get('trajectory_id', -1)) if command else -1,
            "orientation_xyzw": matrix_to_xyzw(rotation_sensor_body @ R_BODY_CAMERA),
            "body_orientation_xyzw": matrix_to_xyzw(rotation_body),
            "depth_unit": "millimeter", "rgb_encoding": "rgb8", "semantic_dtype": "int32",
        })
        sequence += 1
        if args.coarse_coverage_stop and coverage_fraction >= args.coarse_coverage_stop:
            stop_reason = 'coarse_sensor_coverage_goal'
            print(f'Coarse sensor coverage goal reached: {coverage_fraction:.2%}; tiny residual regions are not required', flush=True)
            break
        if sequence % max(1, int(args.hz * 5)) == 0:
            print(
                f"frame={sequence} position={position.round(2).tolist()} "
                f"visited={sorted(visited)} observed={int(observed.sum())} "
                f"fallback={'yes' if fallback_active else 'no'}", flush=True
            )
        time.sleep(max(0.0, period - (time.monotonic() - tick)))

    np.savetxt(
        args.output / "falcon_trajectory.csv", np.asarray(trajectory), delimiter=",",
        header="time_s,x_m,y_m,z_m,yaw_rad", comments="",
    )
    np.savez_compressed(args.output / "sensed_free.npz", sensed_free_zyx=sensed_free)
    np.savetxt(args.output / 'sensor_coverage.csv', np.asarray(coverage_history).reshape(-1,3),
               delimiter=',',header='time_s,observed_safe_xy_fraction,observed_cells',comments='')
    result = {
        "format": "pre_map_vln.real_pcd_falcon_preview.v1",
        "offline_only": True,
        "sensor_model": "simulated_mid360_pointcloud" if args.sensor_model == "mid360" else "simulated_pinhole_depth",
        "sensor_boundary_policy": "physical_snapshot_geometry_without_virtual_geofence",
        "frame_rate_hz": args.hz,
        "snapshot": str(args.snapshot.resolve()),
        "excluded_rooms": args.exclude_room,
        "boundary": str(args.boundary.resolve()) if args.boundary else None,
        "boundary_only": bool(args.boundary_only),
        "height_limits_m": [args.min_z, args.max_z],
        "motion_min_clearance_m": args.motion_min_clearance,
        "coverage_fallback": bool(args.coverage_fallback),
        "command_following": args.command_following,
        "stop_reason": stop_reason,
        "coarse_coverage_goal": args.coarse_coverage_stop,
        "observed_safe_xy_fraction": coverage_fraction,
        "frontier_completion_claimed": False,
        "command_diagnostics": command_diagnostics,
        "rejection_reasons": rejection_reasons,
        "rejection_examples": rejection_examples,
        "walkable_cells_at_1_1m": int(walkable.sum()),
        "observed_free_cells_proxy": int(observed.sum()),
        "observed_free_fraction_proxy": float(observed.sum() / max(1, walkable.sum())),
        "fallback_segments": fallback_segments,
        "target_rooms": sorted(target_rooms),
        "visited_rooms": sorted(visited),
        "all_target_rooms_visited": bool(target_rooms) and target_rooms <= visited,
        "frames": sequence,
        "duration_s": trajectory[-1][0] if trajectory else 0.0,
        "trajectory_csv": str((args.output / "falcon_trajectory.csv").resolve()),
        "map_epoch_uuid": metadata.get("identity", {}).get("map_epoch_uuid"),
    }
    atomic_json(args.output / "result.json", result)
    print(json.dumps({k:v for k,v in result.items() if k != 'rejection_examples'}, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
