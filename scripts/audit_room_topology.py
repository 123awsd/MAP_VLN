#!/usr/bin/env python3
"""Independent region connectivity and wall/domain audit; no room-count truth.

Disconnected regions are review candidates, not automatically errors: a shared
logical stairwell can have multiple observed components. A clean report cannot
prove that unseen rooms, false merges or incorrect walls have been resolved.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import label


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('maps', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    reports = []
    for path in args.maps:
        with np.load(path) as data:
            regions = data['labels']
            wall, domain = data['wall'].astype(bool), data['domain'].astype(bool)
            resolution = float(data['resolution'])
            origin = data['origin'].tolist()
        if regions.shape != wall.shape or regions.shape != domain.shape or resolution <= 0:
            raise ValueError(f'Inconsistent map geometry: {path}')
        components = []
        for rid in np.unique(regions[regions > 0]):
            connected, count = label(regions == rid)
            areas = np.sort(np.bincount(connected.ravel())[1:] * resolution**2)[::-1]
            components.append({'region_id': int(rid), 'area_m2': float(areas.sum()),
                               'connected_components_4': count,
                               'component_areas_m2': areas.tolist(),
                               'multiple_components_over_050m2': int((areas >= .5).sum()) > 1})
        reports.append({'source': str(path), 'resolution_m': resolution, 'origin': origin,
                        'positive_region_count': len(components),
                        'labeled_wall_overlap_cells': int(((regions > 0) & wall).sum()),
                        'labeled_outside_domain_cells': int(((regions > 0) & ~domain).sum()),
                        'unlabeled_nonwall_domain_cells': int((domain & ~wall & (regions <= 0)).sum()),
                        'regions': components})
    result = {'format': 'pre_map_vln.room_topology_audit.v1', 'maps': reports,
              'full_room_correctness_proven': False,
              'limitations': 'Independent connectivity/consistency only; no semantic or scene truth, no inference from equal stair areas. False splits/merges require geometric portal and visual review.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    for report in reports:
        print(Path(report['source']).name, 'regions', report['positive_region_count'],
              'wall overlap', report['labeled_wall_overlap_cells'],
              'disconnected large components', sum(r['multiple_components_over_050m2'] for r in report['regions']))


if __name__ == '__main__':
    main()
