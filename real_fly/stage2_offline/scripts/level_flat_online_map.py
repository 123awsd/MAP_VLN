#!/usr/bin/env python3
"""Make an isolated gravity-level preview from an existing online mapping run.

The input run is never modified. The output uses the same RGB/depth Bag (via a
hard link) and rotates all world-frame geometry and odometry consistently.
This is intended for a site independently known to have a single flat floor.
"""

import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

import numpy as np
import rosbag


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def level_rotation(a, b):
    normal = np.array((-a, -b, 1.0), dtype=np.float64)
    normal /= np.linalg.norm(normal)
    up = np.array((0.0, 0.0, 1.0))
    cross = np.cross(normal, up)
    quat = np.r_[1.0 + np.dot(normal, up), cross]
    quat /= np.linalg.norm(quat)  # w, x, y, z
    w, x, y, z = quat
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return rotation, quat


def rotate_odom(msg, rotation, quat):
    pose = msg.pose.pose
    point = pose.position
    point.x, point.y, point.z = map(float, rotation @ [point.x, point.y, point.z])
    q = pose.orientation
    w, x, y, z = quat
    ow, ox, oy, oz = q.w, q.x, q.y, q.z
    q.w = float(w*ow - x*ox - y*oy - z*oz)
    q.x = float(w*ox + x*ow + y*oz - z*oy)
    q.y = float(w*oy - x*oz + y*ow + z*ox)
    q.z = float(w*oz + x*oy - y*ox + z*ow)
    return msg


def rotate_cloud(msg, rotation):
    if msg.is_bigendian or msg.row_step != msg.width * msg.point_step:
        raise ValueError('unsupported PointCloud2 layout')
    fields = {field.name: field for field in msg.fields}
    names = ('x', 'y', 'z')
    if any(name not in fields or fields[name].datatype != 7 for name in names):
        raise ValueError('PointCloud2 must have float32 x/y/z')
    buf = bytearray(msg.data)
    dtype = np.dtype({'names': names,
                      'formats': ['<f4'] * 3,
                      'offsets': [fields[name].offset for name in names],
                      'itemsize': msg.point_step})
    cloud = np.frombuffer(buf, dtype=dtype, count=msg.width * msg.height)
    xyz = np.column_stack([cloud[name] for name in names])
    rotated = xyz @ rotation.T
    for column, name in enumerate(names):
        cloud[name] = rotated[:, column].astype(np.float32)
    normals = ('normal_x', 'normal_y', 'normal_z')
    if all(name in fields and fields[name].datatype == 7 for name in normals):
        normal_dtype = np.dtype({'names': normals, 'formats': ['<f4'] * 3,
                                 'offsets': [fields[name].offset for name in normals],
                                 'itemsize': msg.point_step})
        normal_cloud = np.frombuffer(buf, dtype=normal_dtype, count=msg.width * msg.height)
        normal = np.column_stack([normal_cloud[name] for name in normals]) @ rotation.T
        for column, name in enumerate(normals):
            normal_cloud[name] = normal[:, column].astype(np.float32)
    msg.data = bytes(buf)
    return msg


def rotate_pcd(source, target, rotation):
    with source.open('rb') as original, target.open('wb') as output:
        header = b''
        while True:
            line = original.readline()
            if not line:
                raise ValueError('PCD has no DATA line')
            header += line
            if line.strip() == b'DATA binary':
                break
        for required in (b'FIELDS x y z intensity', b'SIZE 4 4 4 4', b'TYPE F F F F'):
            if required not in header:
                raise ValueError('expected binary XYZI float32 PCD')
        output.write(header)
        while True:
            block = original.read(16 * 250000)
            if not block:
                break
            if len(block) % 16:
                raise ValueError('PCD body is not a multiple of 16 bytes')
            values = np.frombuffer(block, dtype='<f4').copy().reshape(-1, 4)
            values[:, :3] = values[:, :3] @ rotation.T
            output.write(values.tobytes())


def rotate_mapping_bag(source, target, rotation, quat):
    with rosbag.Bag(str(source), 'r') as original, rosbag.Bag(str(target), 'w', compression='lz4') as output:
        for topic, msg, stamp in original.read_messages():
            if topic == '/Odometry':
                msg = rotate_odom(msg, rotation, quat)
            elif topic == '/cloud_registered':
                msg = rotate_cloud(msg, rotation)
            output.write(topic, msg, stamp)


def rotate_pose_bag(source, target, rotation, quat):
    with rosbag.Bag(str(source), 'r') as original, rosbag.Bag(str(target), 'w', compression='lz4') as output:
        for topic, msg, stamp in original.read_messages(topics=['/ekf_quat/ekf_odom']):
            output.write(topic, rotate_odom(msg, rotation, quat), stamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plane-a', type=float, required=True, help='z = a*x + b*y + c')
    parser.add_argument('--plane-b', type=float, required=True)
    parser.add_argument('--plane-c', type=float, required=True)
    args = parser.parse_args()
    source, target = args.source.resolve(), args.output.absolute()
    if target.exists() or source == target:
        raise SystemExit('refusing to overwrite an existing run')
    if not np.isfinite([args.plane_a, args.plane_b, args.plane_c]).all():
        raise SystemExit('plane coefficients must be finite')
    source_online = source / 'online'
    completion = json.loads((source_online / 'completion.json').read_text())
    if completion.get('status') != 'complete':
        raise SystemExit('source online map is incomplete')
    for name, digest in completion['sha256'].items():
        if sha256(source_online / name) != digest:
            raise SystemExit('source hash mismatch: ' + name)
    rotation, quat = level_rotation(args.plane_a, args.plane_b)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.level-flat-', dir=target.parent) as temporary:
        staging = Path(temporary)
        online, raw = staging / 'online', staging / 'raw'
        online.mkdir()
        raw.mkdir()
        for name in ('ekf_reference.json', 'fastlio.yaml', 'lidar_camera.json'):
            shutil.copy2(source_online / name, online / name)
        rotate_pcd(source_online / 'dense_map.pcd', online / 'dense_map.pcd', rotation)
        rotate_mapping_bag(source_online / 'mapping_outputs.bag', online / 'mapping_outputs.bag', rotation, quat)
        rotate_pose_bag(source / 'raw/sensors.bag', online / 'leveled_pose.bag', rotation, quat)
        # RGB/depth images have no world coordinates; reuse the immutable Bag.
        os.link(source / 'raw/sensors.bag', raw / 'sensors.bag')
        shutil.copy2(source / 'raw/completion.json', raw / 'completion.json')
        manifest = (source / 'manifest.txt').read_text()
        manifest = manifest.replace('run_id=' + source.name, 'run_id=' + target.name, 1)
        (staging / 'manifest.txt').write_text(manifest + 'source_run_id=' + source.name + '\n'
                                             + 'world_frame_leveled=true\n')
        completion['run_id'] = target.name
        completion['sha256'] = {name: sha256(online / name) for name in
                                ('dense_map.pcd', 'mapping_outputs.bag', 'leveled_pose.bag',
                                 'ekf_reference.json', 'fastlio.yaml', 'lidar_camera.json')}
        completion['source_run_id'] = source.name
        completion['world_frame_leveling'] = {'plane_z': [args.plane_a, args.plane_b, args.plane_c],
                                               'rotation': rotation.tolist(),
                                               'status': 'offline_preview_only'}
        (online / 'completion.json').write_text(json.dumps(completion, indent=2) + '\n')
        os.rename(staging, target)
    print(json.dumps({'output': str(target), 'source': str(source),
                      'floor_at_origin_m': args.plane_c,
                      'tilt_deg': float(np.degrees(np.arctan(np.hypot(args.plane_a, args.plane_b)))),
                      'status': 'offline_preview_only'}, indent=2))


if __name__ == '__main__':
    main()
