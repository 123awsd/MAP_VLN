#!/usr/bin/env python3
"""Export this run's actual RGB/pose and old-style dense depth samples for video.

ROS/container utility only; no simulation, planning, GT, or synthetic observations.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag

parser = argparse.ArgumentParser()
parser.add_argument('trial', type=Path)
parser.add_argument('--frames', type=int, default=800)
args = parser.parse_args()
out = args.trial/'video_cache'
out.mkdir(exist_ok=False)
poses, depth_stamps = {}, []
with rosbag.Bag(str(args.trial/'trial.bag')) as bag:
    for topic, msg, _ in bag.read_messages(topics=['/uav_simulator/sensor_pose', '/uav_simulator/depth_image']):
        stamp = msg.header.stamp.to_nsec()
        if topic.endswith('sensor_pose'):
            p, q = msg.transform.translation, msg.transform.rotation
            assert msg.child_frame_id == 'camera_optical'
            poses[stamp] = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        else:
            depth_stamps.append(stamp)
stamps = sorted(set(depth_stamps))
assert len(stamps) >= args.frames and all(t in poses for t in stamps)
selected = [stamps[i] for i in np.linspace(0, len(stamps)-1, args.frames).round().astype(int)]
lookup = {stamp: i for i, stamp in enumerate(selected)}
first = stamps[0]
targets = (np.array(selected, dtype=np.int64)-first)*1e-9
trajectory_times = (np.array(stamps, dtype=np.int64)-first)*1e-9
camera = np.column_stack([targets, np.array([poses[t] for t in selected])])
np.save(out/'camera_pose.npy', camera)
np.save(out/'trajectory.npy', np.column_stack([trajectory_times, np.array([poses[t][:3] for t in stamps])]))
clouds = {f'new_{i:03d}': [] for i in range(args.frames)}
saved_rgb, rng, last = set(), np.random.default_rng(42), -1e30
sampled_depth = 0

def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

with rosbag.Bag(str(args.trial/'trial.bag')) as bag:
    for topic, msg, _ in bag.read_messages(topics=['/habitat/rgb', '/uav_simulator/depth_image']):
        stamp = msg.header.stamp.to_nsec()
        if topic == '/habitat/rgb':
            if stamp not in lookup:
                continue
            i = lookup[stamp]
            image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            image = image[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                image = image[:, :, ::-1]
            else:
                assert msg.encoding == 'bgr8', msg.encoding
            assert cv2.imwrite(str(out/f'rgb_{i:03d}.jpg'), image)
            saved_rgb.add(i)
            continue
        elapsed = (stamp-first)*1e-9
        if elapsed-last < .25 or stamp not in poses:
            continue
        depth = np.frombuffer(msg.data, '<u2').reshape(msg.height, msg.step//2)[:, :msg.width]*.001
        v, u = np.where((depth > 0) & (depth <= 5) & (rng.random(depth.shape) < .05))
        z = depth[v, u]
        pose = poses[stamp]
        points = np.column_stack([(u-320)*z/320, (v-240)*z/320, z])@rotation(pose[3:]).T+pose[:3]
        points = points[points[:, 2] < 8.05].astype(np.float32)
        i = min(args.frames-1, int(np.searchsorted(targets, elapsed, side='left')))
        clouds[f'new_{i:03d}'].append(points)
        sampled_depth += 1
        last = elapsed
assert len(saved_rgb) == args.frames
clouds = {key: np.concatenate(parts) if parts else np.empty((0, 3), np.float32) for key, parts in clouds.items()}
np.savez_compressed(out/'increments.npz', targets=targets, **clouds)
raw = np.concatenate(list(clouds.values()))
np.savez_compressed(out/'timeline.npz', trajectory=np.array([poses[t][:3] for t in stamps], np.float32),
                    trajectory_times=trajectory_times, snapshot_times=targets,
                    bag_start=0., bag_end=targets[-1], cloud_100=raw[::10])
manifest = {'source_bag': str(args.trial/'trial.bag'), 'source_capture_duration_s': float(targets[-1]),
            'rgb_frames': args.frames, 'actual_camera_poses': len(stamps), 'sampled_depth_frames': sampled_depth,
            'depth_points': len(raw), 'sampling': 'original dense style: random5%, seed42, minimum .25s, optical depth0–5m, display z<8.05m',
            'camera': 'actual acquired RGB and optical pose, exact header timestamp; no yaw rerender',
            'progressive_cloud': 'each increment contains only observations at or before that keyframe',
            'playback': 'full recorded episode uniformly accelerated to40s at30fps; no failure or hold segments removed',
            'rgb_source_frame_indices': np.linspace(0, len(stamps)-1, args.frames).round().astype(int).tolist()}
(out/'manifest.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps({k: v for k, v in manifest.items() if k != 'rgb_source_frame_indices'}), flush=True)
