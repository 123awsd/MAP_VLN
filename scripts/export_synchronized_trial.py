#!/usr/bin/env python3
"""Export actual depth geometry and a fixed RGB-D subset from an immutable bag.

ROS-only. Intrinsics must be supplied from the acquisition configuration.
Voxel reduction retains measured points; it neither fills holes nor clips roofs.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rosbag


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--intrinsics', required=True, nargs=4, type=float,
                        metavar=('FX', 'FY', 'CX', 'CY'))
    parser.add_argument('--episode-frames', type=int, default=40)
    parser.add_argument('--pixel-step', type=int, default=6)
    parser.add_argument('--voxel-m', type=float, default=.04)
    parser.add_argument('--max-depth-m', type=float, default=5.)
    parser.add_argument('--skip-unmatched-poses', action='store_true',
                        help='Explicitly exclude depth frames without exact-stamp pose; record exclusions in manifest')
    parser.add_argument('--episode-only', action='store_true', help='Export RGB-D subset without recomputing geometry')
    parser.add_argument('--keyframe-translation-m', type=float, default=None)
    parser.add_argument('--keyframe-rotation-deg', type=float, default=15.)
    args = parser.parse_args()
    if min(args.intrinsics[:2]) <= 0 or args.voxel_m <= 0 or args.pixel_step < 1 or args.episode_frames < 1:
        parser.error('Focal lengths, voxel size, pixel step and frame count must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    episode = args.output / 'episode'
    episode.mkdir()
    poses = {}
    depths = []
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, m, _ in bag.read_messages(topics=['/uav_simulator/sensor_pose', '/uav_simulator/depth_image']):
            stamp = m.header.stamp.to_nsec()
            if topic.endswith('sensor_pose'):
                p, q = m.transform.translation, m.transform.rotation
                poses[stamp] = (np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
            else:
                depths.append(stamp)
    missing_pose_stamps = [s for s in depths if s not in poses]
    original_depth_count = len(depths)
    if not depths or (missing_pose_stamps and not args.skip_unmatched_poses):
        raise RuntimeError('Empty depth stream or missing exact-stamp optical pose')
    depths = [s for s in depths if s in poses]
    if not depths:
        raise RuntimeError('No exactly aligned depth frames remain')
    depths = np.asarray(depths, dtype=np.int64)
    candidates=depths.tolist()
    if args.keyframe_translation_m is not None:
        if args.keyframe_translation_m<=0 or args.keyframe_rotation_deg<=0:
            parser.error('Keyframe motion thresholds must be positive')
        candidates=[int(depths[0])]
        for stamp in depths[1:]:
            position,q=poses[int(stamp)];last_position,last_q=poses[candidates[-1]]
            angle=np.degrees(2*np.arccos(np.clip(abs(np.dot(q,last_q))/(np.linalg.norm(q)*np.linalg.norm(last_q)),0,1)))
            if np.linalg.norm(position-last_position)>=args.keyframe_translation_m or angle>=args.keyframe_rotation_deg:
                candidates.append(int(stamp))
    chosen=np.unique(np.rint(np.linspace(0,len(candidates)-1,min(len(candidates),args.episode_frames))).astype(int))
    selected={candidates[i]for i in chosen}
    fx, fy, cx, cy = args.intrinsics
    rgb, chosen_depth, observed, coarse = {}, {}, {}, set()
    rows = []
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, m, _ in bag.read_messages(topics=['/habitat/rgb', '/uav_simulator/depth_image']):
            stamp = m.header.stamp.to_nsec()
            if topic == '/habitat/rgb':
                if stamp in selected:
                    if m.encoding != 'rgb8':
                        raise RuntimeError('RGB stream encoding must be rgb8')
                    rgb[stamp] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width*3].reshape(m.height, m.width, 3).copy()
                continue
            if m.encoding != '16UC1' or m.is_bigendian:
                raise RuntimeError('Depth stream must contain little-endian 16UC1 millimetres')
            if stamp not in poses:
                continue
            d = np.frombuffer(m.data, '<u2').reshape(m.height, m.step//2)[:, :m.width] * .001
            if stamp in selected:
                chosen_depth[stamp] = d.astype(np.float32)
            if args.episode_only:
                continue
            v, u = np.mgrid[0:m.height:args.pixel_step, 0:m.width:args.pixel_step]
            v, u = v.ravel(), u.ravel()
            z = d[v, u]
            keep = np.isfinite(z) & (z > .1) & (z <= args.max_depth_m)
            v, u, z = v[keep], u[keep], z[keep]
            p, q = poses[stamp]
            points = np.c_[(u-cx)*z/fx, (v-cy)*z/fy, z] @ rotation(q).T + p
            keys = np.floor(points/args.voxel_m).astype(np.int32)
            _, first = np.unique(keys, axis=0, return_index=True)
            for key, point in zip(keys[first], points[first]):
                observed.setdefault(tuple(key), point.astype(np.float32))
            cells = set(map(tuple, np.floor(points/.2).astype(np.int32)))
            new = cells - coarse
            coarse.update(cells)
            rows.append([stamp, (stamp-int(depths[0]))*1e-9, *p, len(points), len(new), len(coarse)])
    missing = selected - rgb.keys()
    if missing:
        raise RuntimeError(f'{len(missing)} selected depth frames lack exact-stamp RGB')
    for i, stamp in enumerate(sorted(selected)):
        p, q = poses[stamp]
        np.savez_compressed(episode/f'frame_{i:06d}.npz', rgb=rgb[stamp], depth_m=chosen_depth[stamp],
                            position=p.astype(np.float32), orientation_xyzw=q.astype(np.float32), time_ns=np.int64(stamp))
    if not args.episode_only:
        np.savez_compressed(args.output/'observed_points.npz', points=np.asarray(list(observed.values()), dtype=np.float32))
        with (args.output/'frame_evidence.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['stamp_ns', 'elapsed_s', 'x', 'y', 'z', 'sampled_surface_points', 'new_surface_voxels_020m', 'cumulative_surface_voxels_020m'])
            writer.writerows(rows)
    manifest = dict(format='pre_map_vln.habitat_episode.v1', source_bag=str(args.bag),
                    fx=fx, fy=fy, cx=cx, cy=cy, coordinate_frame='falcon_world_z_up_camera_optical',
                    depth_unit='metre', rgb_encoding='rgb8', exact_stamp_rgb_depth_pose=True,
                    selected_stamp_ns=sorted(selected), sampling='bounded evenly spaced motion-keyframe indices' if args.keyframe_translation_m is not None else 'fixed evenly spaced depth frame indices',
                    keyframe_translation_m=args.keyframe_translation_m,keyframe_rotation_deg=args.keyframe_rotation_deg,
                    candidate_keyframes=len(candidates),selected_frames=len(selected),geometry_exported=not args.episode_only,
                    geometry_voxel_m=args.voxel_m, geometry_pixel_step=args.pixel_step,
                    max_depth_m=args.max_depth_m, depth_frames=len(depths),
                    original_depth_frames=original_depth_count,
                    excluded_depth_stamps_without_exact_pose=missing_pose_stamps,
                    observed_geometry_points=len(observed), scene_truth_used=False,
                    limitations='Surface novelty is an observation proxy, not completeness or semantic accuracy; no roof clipping.')
    (episode/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({k: v for k, v in manifest.items() if k != 'selected_stamp_ns'}, indent=2))


if __name__ == '__main__':
    main()
