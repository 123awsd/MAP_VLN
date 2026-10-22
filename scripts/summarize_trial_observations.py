#!/usr/bin/env python3
"""Per-floor observation evidence, not a completeness/accuracy estimator.

Height bands are supplied for offline assessment only, never planner input.
Surface points may have been seen from another floor. Camera dwell and surface
support are therefore reported independently. Stationary scans are not assumed
to be wasted if sampled surface novelty remains positive.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('export', type=Path)
    parser.add_argument('--height-edges', type=float, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    edges = np.asarray(args.height_edges)
    if len(edges) < 2 or not np.all(np.diff(edges) > 0):
        parser.error('Height edges must strictly increase')
    frames = np.atleast_1d(np.genfromtxt(args.export/'frame_evidence.csv', delimiter=',', names=True))
    points = np.load(args.export/'observed_points.npz')['points']
    p = np.column_stack([frames[k] for k in ('x', 'y', 'z')])
    time = frames['elapsed_s']
    dt = np.diff(time)
    length = np.linalg.norm(np.diff(p, axis=0), axis=1)
    stationary = length < .01
    novelty = frames['new_surface_voxels_020m'][1:]
    records = []
    for i, (lo, hi) in enumerate(zip(edges[:-1], edges[1:]), 1):
        camera = (p[:, 2] >= lo) & (p[:, 2] < hi)
        # Exclude crossing segments from per-floor travel, but retain globally.
        interval = camera[:-1] & camera[1:]
        surface = points[(points[:, 2] >= lo) & (points[:, 2] < hi)]
        voxels = np.unique(np.floor(surface/.2).astype(np.int32), axis=0)
        records.append({
            'floor_band_index': i, 'z_range_m': [float(lo), float(hi)],
            'camera_depth_frames': int(camera.sum()),
            'first_camera_time_s': float(time[np.flatnonzero(camera)[0]]) if camera.any() else None,
            'within_band_camera_path_m': float(length[interval].sum()),
            'within_band_time_s': float(dt[interval].sum()),
            'stationary_time_s': float(dt[interval & stationary].sum()),
            'stationary_zero_sampled_novelty_time_s': float(dt[interval & stationary & (novelty == 0)].sum()),
            'sampled_surface_voxels_020m': len(voxels),
            'sampled_surface_xy_columns_020m': len(np.unique(voxels[:, :2], axis=0)),
        })
    result = {'source_export': str(args.export), 'assessment_only_height_bands': True,
              'depth_duration_s': float(time[-1]-time[0]),
              'actual_depth_pose_path_m': float(length.sum()),
              'between_band_or_outside_path_m': float(length.sum()-sum(r['within_band_camera_path_m'] for r in records)),
              'floors': records, 'full_coverage_proven': False,
              'limitations': 'Fixed sampled depth evidence; stationary novelty can miss improvements between sampled pixels. Bands are offline conventions, not automatically inferred floors. Surface support is not coverage of all rooms/walls.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
