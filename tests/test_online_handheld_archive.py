"""Run under the real-fly Noetic image (ROS messages, NumPy, SciPy)."""
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'real_fly/stage2_offline/scripts'))
sys.path.insert(0, str(ROOT / 'real_fly/stage1_exploration/scripts'))
from record_online_map import Archive, packed_points
from export_real_boxer_episode import ekf_to_body_pose, rotation_matrix, Rotation, load_pcd_xyz

REFERENCE = dict(Rr_i=[1, 0, 0, 0], tr_i=[0, 0, .08], imu_trans_x=0,
                 imu_trans_y=0, imu_trans_z=0, offset_px=0, offset_py=0, offset_pz=.1)


class OnlineArchiveTest(unittest.TestCase):
    def test_ekf_roundtrip_with_nonzero_rotation_and_offsets(self):
        ref = dict(REFERENCE)
        ref.update(Rr_i=[.9238795325, 0, 0, .3826834324], imu_trans_x=.03)
        pose = np.eye(4)
        pose[:3, :3] = rotation_matrix(Rotation.from_euler('xyz', [.2, -.1, .5]))
        pose[:3, 3] = [1, 2, 3]
        q = np.array(ref['Rr_i'])
        ri = pose[:3, :3] @ rotation_matrix(Rotation.from_quat(q[[1, 2, 3, 0]])).T
        published = pose.copy()
        published[:3, :3] = ri
        published[:3, 3] = pose[:3, 3] - ri @ np.array(ref['tr_i']) + ri @ np.array([.03, 0, 0]) + [0, 0, .1]
        np.testing.assert_allclose(ekf_to_body_pose(published, ref), pose, atol=1e-9)

    def test_dense_map_and_sparse_rays_preserve_actual_samples(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive = Archive(Path(tmp) / 'online', 'trial', .5)
            archive.reference = REFERENCE
            for name in ('ekf_reference.json', 'fastlio.yaml', 'lidar_camera.json'):
                (archive.output / name).write_text(json.dumps(REFERENCE))
            fields = [PointField(n, i * 4, PointField.FLOAT32, 1)
                      for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
            for i in range(20):
                stamp = rospy.Time.from_sec(100 + i * .1)
                msg = point_cloud2.create_cloud(Header(stamp=stamp, frame_id='world'), fields,
                                               [[1, 2, 3, 4], [float('nan'), 0, 0, 0]])
                archive.cloud(msg)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.orientation.w = 1
                archive.odom(odom)
            archive.finish()
            report = json.loads((archive.output / 'completion.json').read_text())
            self.assertEqual(report['points'], 20)
            self.assertEqual(report['ray_scans'], 4)
            xyz = load_pcd_xyz(archive.output / 'dense_map.pcd')
            np.testing.assert_allclose(xyz, np.tile([1, 2, 3], (20, 1)))
            with rosbag.Bag(str(archive.output / 'mapping_outputs.bag')) as bag:
                self.assertEqual(bag.get_message_count('/cloud_registered'), 4)
                self.assertEqual(bag.get_message_count('/Odometry'), 20)

    def test_incomplete_recording_has_no_completion_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive = Archive(Path(tmp) / 'online', 'trial', .5)
            with self.assertRaises(RuntimeError):
                archive.finish()
            self.assertFalse((archive.output / 'completion.json').exists())


if __name__ == '__main__':
    unittest.main()
