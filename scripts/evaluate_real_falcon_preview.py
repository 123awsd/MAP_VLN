#!/usr/bin/env python3
"""Offline sensor-coverage and actual-motion audit; never creates planner goals."""
import argparse
import json
from pathlib import Path
import numpy as np
from matplotlib.path import Path as PolygonPath
from scipy.ndimage import label
from run_pcd_falcon import load_environment, render_mid360


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', type=Path, required=True)
    parser.add_argument('--scene-graph', type=Path, required=True)
    args = parser.parse_args()
    result = json.loads((args.preview / 'result.json').read_text())
    snapshot = Path(result['snapshot'])
    values = load_environment(snapshot, args.scene_graph, set(), Path(result['boundary']))
    metadata, occupancy, esdf, origin, resolution, allowed, *_ = values
    trajectory = np.atleast_2d(np.loadtxt(args.preview / 'falcon_trajectory.csv', delimiter=',', skiprows=1))
    cache = args.preview / 'sensed_free.npz'
    if cache.exists():
        sensed = np.load(cache)['sensed_free_zyx']
        coverage_source = 'actual simulated ray traversals'
    else:
        sensed = np.zeros_like(occupancy, dtype=bool)
        previous = None
        # Ray-casting a subset is a conservative replay approximation, not the
        # complete online scan history. Do not label it exact coverage.
        for row in trajectory:
            position = row[1:4]
            if previous is None or np.linalg.norm(position - previous) >= 0.4:
                render_mid360(occupancy, origin, resolution, allowed, position, sensed)
                previous = position
        coverage_source = 'conservative replay every 0.4 m'
    z = origin[2] + (np.arange(occupancy.shape[0]) + .5) * resolution
    band = (z >= .8-1e-6) & (z <= 1.5+1e-6)
    z_index = int(np.floor((1.1-origin[2])/resolution))
    target = allowed & (occupancy[z_index] == 0) & (esdf[z_index] >= .25)
    components, _ = label(target)
    x0, y0, _ = np.floor((trajectory[0,1:4]-origin)/resolution).astype(int)
    start_component = int(components[y0,x0])
    if start_component:
        target &= components == start_component
    seen = sensed[band].any(axis=0) & target
    # Score coarse 1 m cells only when at least 20% of their safe target cells
    # were observed. Very small (<0.32 m^2) pockets are separately ignored.
    yy, xx = np.indices(target.shape)
    bx = np.floor((origin[0]+(xx+.5)*resolution)).astype(int)
    by = np.floor((origin[1]+(yy+.5)*resolution)).astype(int)
    key = (by-by.min())*(int(bx.max()-bx.min())+1)+(bx-bx.min())
    total = np.bincount(key[target])
    covered = np.bincount(key[seen], minlength=len(total))
    eligible = total >= max(1,int(np.ceil(.32/resolution**2)))
    coarse_seen = eligible & (covered >= .2*total)
    rooms = {}
    xy = np.column_stack((origin[0]+(xx.ravel()+.5)*resolution,
                          origin[1]+(yy.ravel()+.5)*resolution))
    for room in json.loads(args.scene_graph.read_text())['rooms']:
        mask = PolygonPath(np.array(room['polygon_xy_m'])).contains_points(xy).reshape(target.shape) & target
        if mask.sum():
            rooms[room['name']] = {'target_cells':int(mask.sum()),
                                  'observed_fraction':float(seen[mask].mean())}
    positions = trajectory[:,1:4]
    movement = np.linalg.norm(np.diff(positions,axis=0),axis=1)
    # Audit interpolated actual motion, not merely recorded endpoints. This is
    # a static-map check; final sensed history alone is not a temporal proof.
    dense = np.concatenate([
        np.linspace(a,b,max(2,int(np.ceil(length/.01))+1))
        for a,b,length in zip(positions[:-1],positions[1:],movement)
    ]) if len(positions)>1 else positions
    indices = np.floor((dense-origin)/resolution).astype(int)
    in_map = np.all((indices >= 0) & (indices < np.array(occupancy.shape[::-1])),axis=1)
    valid = indices[in_map]
    xi,yi,zi = valid.T
    occ = occupancy[zi,yi,xi]
    distances = esdf[zi,yi,xi]
    continuous_audit = {
        'sample_spacing_max_m':.01,
        'samples':len(dense),
        'outside_map_samples':int((~in_map).sum()),
        'occupied_samples':int((occ==100).sum()),
        'unobserved_unknown_samples':int(((occ==-1)&~sensed[zi,yi,xi]).sum()),
        'outside_allowed_raster_samples':int((~allowed[yi,xi]).sum()),
        'clearance_below_025_samples':int((distances < .25-1e-6).sum()),
        'minimum_snapshot_esdf_m':float(distances.min()) if len(distances) else None,
        'height_min_m':float(dense[:,2].min()),
        'height_max_m':float(dense[:,2].max()),
        'note':'static 1cm-sampled motion audit, not executable MINCO certification',
    }
    dt = np.diff(trajectory[:,0])
    path_cells = np.floor(positions[:,:2]).astype(int)
    changed = np.r_[True,np.any(np.diff(path_cells,axis=0),axis=1)]
    visits = path_cells[changed]
    unique = len(np.unique(visits,axis=0))
    audit = {
        'coverage_source':coverage_source,
        'target_definition':'start-connected known safe cells at z=1.1m; scans in z=0.8..1.5m',
        'duration_s':float(trajectory[-1,0]),
        'path_length_m':float(movement.sum()),
        'stationary_seconds':float(dt[movement < .01].sum()),
        'observed_safe_xy_fraction':float(seen.sum()/max(1,target.sum())),
        'coarse_1m_covered_fraction':float(coarse_seen.sum()/max(1,eligible.sum())),
        'coarse_1m_target_cells':int(eligible.sum()),
        'coarse_1m_covered_cells':int(coarse_seen.sum()),
        'one_metre_cell_entries':len(visits), 'unique_one_metre_cells_visited':unique,
        'repeat_cell_entries':len(visits)-unique,
        'repeat_note':'includes legitimate return transit; NOT redundant frontier selection rate',
        'rooms_informational_only':rooms,
        'gate_rejections':result.get('rejection_reasons',{}),
        'no_truth_goal_fallback':not result['coverage_fallback'],
        'continuous_motion_audit':continuous_audit,
    }
    (args.preview/'coverage_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
    np.savez_compressed(args.preview/'coverage_audit_masks.npz',target_xy=target,seen_xy=seen)
    print(json.dumps(audit,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
