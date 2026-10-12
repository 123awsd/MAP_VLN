#!/usr/bin/env python3
"""Stream registered scans to dense PCD; retain sparse measured rays for offline use.

No publishers/services. Point buffers are bounded by the ROS subscriber queue;
the full map is never accumulated in RAM. completion.json is written last.
"""
import argparse
import hashlib
import json
import os
import shutil
import threading
from pathlib import Path

import numpy as np
import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def file_sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def packed_points(msg):
    fields = {f.name: f for f in msg.fields}
    if msg.is_bigendian or any(fields[k].datatype != 7 for k in ('x', 'y', 'z')):
        raise ValueError('expected little-endian FLOAT32 XYZ cloud')
    output = np.zeros((msg.height * msg.width, 4), dtype='<f4')
    for col, key in enumerate(('x', 'y', 'z', 'intensity')):
        if key not in fields:
            continue
        if fields[key].datatype != 7:
            raise ValueError('expected FLOAT32 ' + key)
        values = np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
                            offset=fields[key].offset,
                            strides=(msg.row_step, msg.point_step))
        output[:, col] = values.reshape(-1)
    return output[np.isfinite(output[:, :3]).all(axis=1)]


class Archive:
    def __init__(self, output, run_id, scan_period):
        self.output = output
        self.run_id = run_id
        self.period = scan_period
        self.lock = threading.Lock()
        self.closed = False
        self.error = None
        self.count = self.scans = self.rays = self.odom_count = 0
        self.first = self.last = None
        self.last_ray = -float('inf')
        self.max_gap = 0.0
        self.reference = None
        output.mkdir(parents=True, exist_ok=False)
        self.data_path = output / 'dense_points.partial'
        self.data = self.data_path.open('xb', buffering=1024 * 1024)
        self.bag = rosbag.Bag(str(output / 'mapping_outputs.bag.partial'), 'w', compression='lz4')

    def cloud(self, msg):
        try:
            with self.lock:
                if self.closed:
                    return
                if msg.header.frame_id not in ('world', 'map'):
                    raise ValueError('registered cloud frame must be world/map')
                points = packed_points(msg)
                self.data.write(points.tobytes())
                self.count += len(points)
                self.scans += 1
                stamp = msg.header.stamp.to_sec()
                if self.first is None:
                    self.first = stamp
                if self.last is not None:
                    self.max_gap = max(self.max_gap, stamp - self.last)
                self.last = stamp
                if stamp - self.last_ray >= self.period:
                    self.bag.write('/cloud_registered', msg, msg.header.stamp)
                    self.last_ray = stamp
                    self.rays += 1
        except Exception as exc:
            self.error = str(exc)
            rospy.logerr('Online map recording failed: %s', exc)
            rospy.signal_shutdown('online map write failed')

    def odom(self, msg):
        try:
            with self.lock:
                if not self.closed:
                    self.bag.write('/Odometry', msg, msg.header.stamp)
                    self.odom_count += 1
        except Exception as exc:
            self.error = str(exc)
            rospy.signal_shutdown('odometry archive failed')

    def capture_reference(self, _event=None):
        keys = ('Rr_i', 'tr_i', 'imu_trans_x', 'imu_trans_y', 'imu_trans_z',
                'offset_px', 'offset_py', 'offset_pz')
        if self.reference is None and all(rospy.has_param('/ekf_quat/' + k) for k in keys):
            self.reference = {k: rospy.get_param('/ekf_quat/' + k) for k in keys}
            (self.output / 'ekf_reference.json').write_text(json.dumps(self.reference, indent=2) + '\n')

    def finish(self):
        with self.lock:
            self.closed = True
            self.data.close()
            self.bag.close()
        if self.error or not self.count or not self.odom_count or self.reference is None:
            raise RuntimeError('Incomplete online map: ' + str(self.error or 'missing cloud/odom/EKF reference'))
        dest = self.output / 'dense_map.pcd'
        partial = self.output / 'dense_map.pcd.partial'
        header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
                  'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n'
                  'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (self.count, self.count)
        with partial.open('xb') as out, self.data_path.open('rb') as src:
            out.write(header.encode('ascii'))
            shutil.copyfileobj(src, out, length=1024 * 1024)
            out.flush()
            os.fsync(out.fileno())
        partial.rename(dest)
        self.data_path.unlink()  # Temporary payload now preserved in the completed PCD.
        (self.output / 'mapping_outputs.bag.partial').rename(self.output / 'mapping_outputs.bag')
        files = {p.name: file_sha(p) for p in (dest, self.output / 'mapping_outputs.bag',
                                              self.output / 'ekf_reference.json',
                                              self.output / 'fastlio.yaml', self.output / 'lidar_camera.json')}
        report = dict(format='pre_map_vln.online_map.v1', run_id=self.run_id, points=self.count,
                      scans=self.scans, ray_scans=self.rays, odometry_messages=self.odom_count,
                      first_stamp=self.first, last_stamp=self.last, max_scan_gap_s=self.max_gap,
                      scan_period_s=self.period, sha256=files, status='complete')
        tmp = self.output / 'completion.json.partial'
        tmp.write_text(json.dumps(report, indent=2) + '\n')
        tmp.rename(self.output / 'completion.json')
        print(json.dumps(report), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--scan-period', type=float, default=0.5)
    parser.add_argument('--fastlio-config', type=Path, required=True)
    parser.add_argument('--camera-calibration', type=Path, required=True)
    args = parser.parse_args()
    rospy.init_node('handheld_online_map_recorder')
    archive = Archive(args.output, args.run_id, args.scan_period)
    shutil.copyfile(args.fastlio_config, args.output / 'fastlio.yaml')
    shutil.copyfile(args.camera_calibration, args.output / 'lidar_camera.json')
    subscribers = [rospy.Subscriber('/cloud_registered', PointCloud2, archive.cloud,
                                  queue_size=2, buff_size=16 * 1024 * 1024),
                   rospy.Subscriber('/Odometry', Odometry, archive.odom, queue_size=100)]
    timer = rospy.Timer(rospy.Duration(1), archive.capture_reference)
    try:
        rospy.spin()
    finally:
        timer.shutdown()
        for subscriber in subscribers:
            subscriber.unregister()
        archive.finish()


if __name__ == '__main__':
    main()
