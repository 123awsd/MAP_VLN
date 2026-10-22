#!/usr/bin/env python3
"""Offline roof audit against measured sloped patches with local XY support.

Never sends roof geometry to the planner. Flat virtual caps are excluded.
No local reference support means unverified, not safe. This is a measured
reference check rather than a complete building-containment certificate.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trajectory-jsonl', type=Path, required=True)
    p.add_argument('--ceiling-points', type=Path, required=True)
    p.add_argument('--roof-planes', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--minimum-sensor-z', type=float, default=5.5)
    p.add_argument('--support-radius', type=float, default=.15)
    a = p.parse_args()
    rows = [json.loads(s) for s in a.trajectory_jsonl.read_text().splitlines() if s.strip()]
    poses = np.asarray([r['position'] for r in rows], dtype=float)
    ceiling = np.load(a.ceiling_points)['ceiling_points']
    planes = json.loads(a.roof_planes.read_text())
    selected = np.flatnonzero(poses[:, 2] >= a.minimum_sensor_z)
    q = poses[selected]
    clearance = np.full(len(q), np.inf)
    normal_clearance = np.full(len(q), np.inf)
    matched = np.zeros(len(q), bool)
    accepted = []
    for i, plane in enumerate(planes):
        if not 10 <= plane['inclination_degrees'] <= 70:
            continue
        n = np.asarray(plane['normal'], dtype=float)
        offset = float(plane['offset'])
        support = ceiling[np.abs(ceiling@n-offset) < .055]
        if not len(support):
            continue
        distance, _ = cKDTree(support[:, :2]).query(q[:, :2])
        near = distance <= a.support_radius
        roof_z = (offset-q[:, :2]@n[:2])/n[2]
        gap = roof_z-q[:, 2]
        clearance[near] = np.minimum(clearance[near], gap[near])
        normal_clearance[near] = np.minimum(normal_clearance[near], gap[near]*abs(n[2])/np.linalg.norm(n))
        matched |= near
        accepted.append(i)
    violations = np.flatnonzero(matched & (clearance < 0))
    report = {
        'trajectory_source': str(a.trajectory_jsonl), 'measured_ceiling_source': str(a.ceiling_points),
        'plane_source': str(a.roof_planes), 'offline_reference_only': True,
        'accepted_sloped_plane_indices': accepted, 'local_support_radius_m': a.support_radius,
        'third_floor_pose_samples': len(q), 'locally_supported_samples': int(matched.sum()),
        'unsupported_samples': int((~matched).sum()),
        'minimum_vertical_roof_gap_m': float(clearance[matched].min()) if matched.any() else None,
        'minimum_plane_normal_gap_m': float(normal_clearance[matched].min()) if matched.any() else None,
        'sensor_above_measured_roof_samples': len(violations),
        'violation_sequences': [rows[selected[i]]['sequence'] for i in violations[:30]],
        'full_building_containment_proven': False,
        'limitations': 'Prior-run measured reference; missing XY support remains unverified. Flat caps excluded, but sloped patch correctness still needs visual/scene review. No full robot shape or all walls are certified.',
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
