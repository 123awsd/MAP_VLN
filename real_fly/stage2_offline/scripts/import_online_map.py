#!/usr/bin/env python3
"""Validate and import a completed online map without rebuilding FAST-LIO."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--input-run-id', required=True)
    p.add_argument('--run-id', required=True)
    args = p.parse_args()
    manifest = json.loads((args.source / 'completion.json').read_text())
    if manifest.get('status') != 'complete' or manifest.get('run_id') != args.input_run_id:
        raise SystemExit('Online map is incomplete or RUN_ID does not match')
    names = ['dense_map.pcd', 'mapping_outputs.bag', 'ekf_reference.json', 'fastlio.yaml', 'lidar_camera.json']
    for name in names:
        if digest(args.source / name) != manifest['sha256'].get(name):
            raise SystemExit('Online map hash mismatch: ' + name)
    if 'leveled_pose.bag' in manifest['sha256']:
        if digest(args.source / 'leveled_pose.bag') != manifest['sha256']['leveled_pose.bag']:
            raise SystemExit('Online map hash mismatch: leveled_pose.bag')
    if args.output.exists():
        raise SystemExit('Refusing to overwrite online import: ' + str(args.output))
    args.output.mkdir(parents=True)
    for name in names:
        target = 'handheld_map_' + args.run_id + '_complete.pcd' if name == 'dense_map.pcd' else name
        shutil.copyfile(args.source / name, args.output / target)
    # Keep the established downstream file layout. This is the same dense map,
    # not a replay or an independently reconstructed coordinate system.
    shutil.copyfile(args.source / 'dense_map.pcd', args.output / 'global_dense_map.pcd')
    (args.output / 'global_dense_map.json').write_text(json.dumps({
        'source': 'online_registered_scans', 'points': manifest['points'],
        'input_run_id': args.input_run_id, 'online_manifest': manifest,
    }, indent=2) + '\n')
    (args.output / 'online_import.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Imported online map; FAST-LIO replay skipped: ' + str(args.output))


if __name__ == '__main__':
    main()
