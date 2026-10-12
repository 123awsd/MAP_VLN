#!/usr/bin/env python3
"""Audit a frozen offline exploration route and export MINCO guide input.

This is a hybrid preview, not evidence of autonomous frontier completion.
No ROS nodes or flight commands are started.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from matplotlib.path import Path as Polygon


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preview', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--boundary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-time', type=float, help='Keep a useful route prefix; preserve the full simulation CSV')
    args = parser.parse_args()
    metadata = json.loads((args.snapshot / 'metadata.json').read_text())
    grid = np.load(args.snapshot / 'voxel_map.npz')
    origin = np.asarray(metadata['origin_xyz_m'])
    resolution = metadata['resolution_m']
    boundary = json.loads(args.boundary.read_text())
    outer = Polygon(boundary['allowed_polygon_xy_m'])
    exclusions = [Polygon(p['polygon_xy_m']) for p in boundary.get('excluded_polygons', [])]
    rows = np.loadtxt(args.preview / 'falcon_trajectory.csv', delimiter=',', skiprows=1)
    if args.max_time is not None:
        rows = rows[rows[:, 0] <= args.max_time]
    if len(rows) < 2:
        raise SystemExit('Insufficient trajectory samples')
    xyz = rows[:, 1:4]
    points = [xyz[0]]
    for point in xyz[1:]:
        if np.linalg.norm(point - points[-1]) > 1e-5:
            points.append(point)
    points = np.asarray(points)
    # Audit continuous segments, not just the grid centers at their endpoints.
    samples = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        count = max(1, int(np.ceil(np.linalg.norm(b - a) / .01)))
        samples.extend(a + np.arange(1, count + 1)[:, None] / count * (b - a))
    samples = np.asarray(samples)
    cells = np.floor((samples - origin) / resolution).astype(int)
    shape = np.asarray(grid['raw_occupancy_zyx'].shape[::-1])
    inside_grid = ((cells >= 0) & (cells < shape)).all(axis=1)
    if not inside_grid.all():
        raise SystemExit('Route leaves voxel snapshot; no mission exported')
    x, y, z = cells.T
    occupancy = grid['raw_occupancy_zyx'][z, y, x]
    clearance = grid['esdf_zyx_m'][z, y, x]
    legal = outer.contains_points(samples[:, :2])
    for polygon in exclusions:
        legal &= ~polygon.contains_points(samples[:, :2])
    legal &= (samples[:, 2] >= .5) & (samples[:, 2] <= 1.5)
    safe = legal & (occupancy == 0) & (clearance >= .25 - 1e-6)
    report = {
        'offline_only': True, 'route_source': 'falcon_with_known_map_coverage_supplement',
        'frontier_completion_claimed': False,
        'boundary_sha256': hashlib.sha256(args.boundary.read_bytes()).hexdigest(),
        'length_m': float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
        'height_range_m': [float(points[:, 2].min()), float(points[:, 2].max())],
        'unique_motion_points': len(points), 'audit_spacing_m': .01,
        'violations': int((~safe).sum()),
        'minimum_snapshot_esdf_m': float(clearance.min()),
        'esdf_is_not_exact_cube_surface_clearance': True,
    }
    (args.preview / 'route_audit.json').write_text(json.dumps(report, indent=2))
    if not safe.all():
        raise SystemExit(f'Route audit failed: {report}; no mission exported')
    # Remove only collinear points. No new shortcut through unknown geometry.
    guide = [points[0]]
    for i in range(1, len(points) - 1):
        a, b = points[i] - guide[-1], points[i + 1] - points[i]
        if np.linalg.norm(np.cross(a, b)) > 1e-7 or np.dot(a, b) <= 0:
            guide.append(points[i])
    guide.append(points[-1])
    end = points[-1]
    mission = dict(metadata['identity'])
    mission.update({
        'format': 'pre_map_vln.mission_plan.v1',
        'start_xyz_yaw': rows[0, 1:5].tolist(),
        'real_start_source': 'offline_exploration_preview',
        'start_pose_explicitly_approved': False,
        'exploration_provenance': report,
        'visits': [{'task_id': 'exploration_demo', 'candidate_id': 'route_end',
                    'pose': dict(zip(('x', 'y', 'z', 'yaw'), [*end.tolist(), float(rows[-1, 4])])),
                    'dwell_s': 0.1}],
        'segments': [{'points_xyz_m': np.asarray(guide).tolist()}],
    })
    if args.output.exists():
        raise SystemExit(f'Refusing to overwrite {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(mission, indent=2))
    (args.preview / 'selected_route.csv').write_text(
        'x_m,y_m,z_m\n' + '\n'.join(','.join(map(str, point)) for point in points) + '\n')
    (args.output.parent / 'candidates.json').write_text(json.dumps({'by_task': {}}))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
