#!/usr/bin/env python3
"""Offline ray-observation replay. Publishes only /falcon_preview/*, never control."""
import argparse
import json
import time
import sys
from pathlib import Path
import numpy as np


def lidar_to_world_rotation(yaw, lidar_to_body):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ lidar_to_body


def prepare(preview):
    from run_pcd_falcon import load_environment
    from scipy.spatial import cKDTree
    result = json.loads((preview / 'result.json').read_text())
    snapshot = Path(result['snapshot'])
    scene = snapshot.parent / 'approved_scene_graph.json'
    import yaml
    calibration = snapshot.parent / 'fastlio_complete/fastlio.yaml'
    config = yaml.safe_load(calibration.read_text())
    lidar_to_body = np.asarray(config['mapping']['extrinsic_R'], dtype=float).reshape(3,3)
    if not np.allclose(lidar_to_body.T @ lidar_to_body, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(lidar_to_body),1.,atol=1e-5):
        raise ValueError('Invalid recorded LiDAR-to-body rotation')
    print(f'Using recorded LiDAR-to-body rotation: {calibration}', flush=True)
    _, occupancy, _, origin, resolution, allowed, *_ = load_environment(
        snapshot, scene, set(), Path(result['boundary']))
    trajectory = np.atleast_2d(np.loadtxt(preview / 'falcon_trajectory.csv', delimiter=',', skiprows=1))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'real_fly/stage2_offline/scripts'))
    from downsample_pcd_for_rviz import read_binary_xyz
    source = snapshot.parent / 'fastlio_complete' / f'handheld_map_{result["snapshot"].split("/")[-2]}_complete.pcd'
    if not source.exists(): source = snapshot.parent / 'fastlio_complete/global_dense_map.pcd'
    print(f'Loading dense PCD (visualization only): {source}', flush=True)
    xyz = read_binary_xyz(source)
    # Geofences constrain vehicle positions, NOT what its sensor can see.
    # A hand-drawn polygon often lies just inside a wall. Clipping geometry
    # by that polygon erased the whole wall before ray visibility was tested.
    # Use only a conservative 8m trajectory AABB to reduce memory; the exact
    # radius/FOV/occlusion checks below still decide observations per frame.
    lo, hi = trajectory[:,1:4].min(axis=0)-8., trajectory[:,1:4].max(axis=0)+8.
    valid = np.isfinite(xyz).all(axis=1) & np.all(xyz >= lo, axis=1) & np.all(xyz <= hi, axis=1)
    xyz = xyz[valid]
    # Retain actual measured coordinates, not occupancy-cell centers. No
    # minimum point-support threshold is applied to visualization geometry.
    cells = np.floor(xyz / .08).astype(np.int64)
    cells -= cells.min(axis=0)
    keys = np.ravel_multi_index(cells.T, cells.max(axis=0)+1)
    _, first = np.unique(keys, return_index=True)
    xyz = xyz[first]
    del cells, keys
    print(f'Visualization geometry: {len(xyz)} points, 0.08m downsampling', flush=True)
    tree = cKDTree(xyz)
    first_seen = np.full(len(xyz), np.inf)
    scans, offsets = [], [0]
    for i, row in enumerate(trajectory):
        ids = np.asarray(tree.query_ball_point(row[1:4], 8.), dtype=int)
        delta = xyz[ids] - row[1:4]
        distance = np.linalg.norm(delta, axis=1)
        # Extrinsic maps LiDAR vectors INTO body. Invert body/world rotation
        # before testing the LiDAR's own -7..52 degree vertical field of view.
        # Recorded preview only contains yaw; assume level roll/pitch, not a
        # reconstructed measured attitude. Preserve its existing ray origin.
        local_delta = delta @ lidar_to_world_rotation(row[4], lidar_to_body)
        elevation = np.degrees(np.arctan2(local_delta[:,2], np.linalg.norm(local_delta[:,:2],axis=1)))
        good = (distance >= .15) & (elevation >= -7) & (elevation <= 52)
        ids, local_delta, distance, elevation = ids[good], local_delta[good], distance[good], elevation[good]
        azimuth = np.mod(np.degrees(np.arctan2(local_delta[:,1],local_delta[:,0])),360.)
        bins = np.floor(azimuth).astype(int) + 360*np.floor(elevation+7).astype(int)
        nearest = np.full(360*60, np.inf)
        np.minimum.at(nearest, bins, distance)
        # Angular depth-buffer approximation: a 1-degree bin can contain a
        # slanted surface; allow 8cm surface thickness, not arbitrary distance
        # reveal through walls. This is not a recorded physical lidar scan.
        ids = ids[distance <= nearest[bins] + .08]
        first_seen[ids] = np.minimum(first_seen[ids], row[0])
        scan = xyz[ids]
        scans.append(scan)
        offsets.append(offsets[-1] + len(scan))
        if i % 200 == 0:
            print(f'Rebuilding actual simulated scans: {i}/{len(trajectory)}', flush=True)
    ids = np.flatnonzero(np.isfinite(first_seen))
    order = np.argsort(first_seen[ids], kind='stable')
    ids = ids[order]
    points = xyz[ids]
    output = preview / 'rviz_replay.npz'
    np.savez_compressed(output, trajectory=trajectory, points=points,
                        first_seen=first_seen[ids], scans=np.concatenate(scans),
                        offsets=np.array(offsets),
                        geometry_version=np.array(4), source_pcd=np.array(str(source)),
                        lidar_to_body=lidar_to_body, calibration_yaml=np.array(str(calibration)),
                        geometry_note=np.array('dense PCD; NO geofence or fixed height geometry clipping; recorded LiDAR-to-body rotation plus trajectory yaw; assumed level body; unchanged ray origin; 8cm sampling; approximate 1deg angular occlusion; 8m range; LiDAR-frame -7..52deg'),
                        boundary_json=Path(result['boundary']).read_text())
    print(f'Saved {len(trajectory)} frames, {len(points)} progressively observed PCD points: {output}')


def play(cache, speed, once):
    import rospy
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Path as RosPath
    from sensor_msgs.msg import PointCloud2, PointField
    from visualization_msgs.msg import Marker, MarkerArray
    from std_srvs.srv import SetBool, SetBoolResponse
    data = np.load(cache)
    trajectory, points, first_seen = data['trajectory'], data['points'], data['first_seen']
    scans, offsets = data['scans'], data['offsets']
    boundary = json.loads(str(data['boundary_json']))
    rospy.init_node('falcon_offline_exploration_replay')
    pubs = {name: rospy.Publisher('/falcon_preview/' + name, msg, queue_size=1, latch=True)
            for name, msg in [('observed_map', PointCloud2), ('current_scan', PointCloud2),
                              ('trajectory', RosPath), ('markers', MarkerArray)]}
    fields = [PointField(name, i * 4, PointField.FLOAT32, 1) for i, name in enumerate('xyz')]
    paused = [False]
    def pause(req):
        paused[0] = req.data
        return SetBoolResponse(True, 'paused' if req.data else 'playing')
    rospy.Service('/falcon_preview/pause', SetBool, pause)
    def cloud(array, header):
        a = np.ascontiguousarray(array, dtype='<f4')
        return PointCloud2(header=header, height=1, width=len(a), fields=fields,
                          is_bigendian=False, point_step=12, row_step=len(a)*12,
                          data=a.tobytes(), is_dense=True)
    deadline = time.monotonic() + 30
    while not rospy.is_shutdown() and not pubs['observed_map'].get_num_connections() and time.monotonic() < deadline:
        time.sleep(.1)
    while not rospy.is_shutdown():
        path = RosPath()
        last_map_time = -np.inf
        for i, row in enumerate(trajectory):
            delay = 0 if i == 0 else (row[0] - trajectory[i-1, 0]) / speed
            remaining = delay
            while remaining > 0 or paused[0]:
                if rospy.is_shutdown(): return
                tick = time.monotonic()
                time.sleep(min(.02, remaining) if remaining > 0 else .02)
                if not paused[0]: remaining -= time.monotonic() - tick
            if rospy.is_shutdown(): return
            header = Header(stamp=rospy.Time.now(), frame_id='world')
            n = np.searchsorted(first_seen, row[0], side='right')
            # Large dense geometry is updated at 1Hz, while scan, pose and
            # trajectory retain the recorded rate. Do not push a million-point
            # accumulated cloud through RViz five times each second.
            if row[0] - last_map_time >= 1.0 or i == len(trajectory)-1:
                pubs['observed_map'].publish(cloud(points[:n], header))
                last_map_time = row[0]
            pubs['current_scan'].publish(cloud(scans[offsets[i]:offsets[i+1]], header))
            pose = PoseStamped(header=header)
            pose.pose.position = Point(*row[1:4])
            pose.pose.orientation.z = np.sin(row[4]/2)
            pose.pose.orientation.w = np.cos(row[4]/2)
            path.header = header
            path.poses.append(pose)
            pubs['trajectory'].publish(path)
            markers = MarkerArray()
            vehicle = Marker(header=header, ns='vehicle', id=0, type=Marker.ARROW, action=Marker.ADD)
            vehicle.pose = pose.pose
            vehicle.scale.x, vehicle.scale.y, vehicle.scale.z = .6, .12, .12
            vehicle.color.r, vehicle.color.g, vehicle.color.a = 1., .65, 1.
            markers.markers.append(vehicle)
            polygons = [boundary['allowed_polygon_xy_m']] + [
                item.get('polygon_xy_m', []) if isinstance(item, dict) else item
                for item in boundary.get('excluded_polygons', [])]
            for j, polygon in enumerate(polygons):
                if len(polygon) < 3: continue
                marker = Marker(header=header, ns='boundary', id=j, type=Marker.LINE_STRIP, action=Marker.ADD)
                marker.pose.orientation.w = 1
                marker.scale.x = .06
                marker.color.r, marker.color.b, marker.color.a = (0.6 if j == 0 else 1.), (1. if j == 0 else .1), 1.
                marker.points = [Point(x, y, .8) for x, y in polygon + [polygon[0]]]
                markers.markers.append(marker)
            label = Marker(header=header, ns='status', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.position = Point(row[1], row[2], row[3] + .6)
            label.pose.orientation.w = 1
            label.scale.z = .22
            label.color.r = label.color.g = label.color.b = label.color.a = 1.
            label.text = f'{row[0]:.1f}s / {trajectory[-1,0]:.1f}s | {speed:g}x'
            markers.markers.append(label)
            pubs['markers'].publish(markers)
        if once:
            rospy.loginfo('Replay finished; holding final visualization. Ctrl-C to exit.')
            rospy.spin()
            return
        rospy.loginfo('Restarting replay with empty observed map/path')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['prepare', 'play'])
    parser.add_argument('path', type=Path)
    parser.add_argument('--speed', type=float, default=1.)
    parser.add_argument('--loop', action='store_true')
    args = parser.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0: parser.error('speed must be positive')
    prepare(args.path) if args.mode == 'prepare' else play(args.path, args.speed, not args.loop)
