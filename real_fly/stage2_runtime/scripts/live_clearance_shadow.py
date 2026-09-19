#!/usr/bin/env python3
"""Subscribe only: never publishes ROS messages or calls services.

Requires a live (not accumulated map) PointCloud2 already in world frame.
Velocity is explicitly world-frame EKF velocity; confirm this for other sources.
"""
import argparse
import json
import time
import threading
import resource
import numpy as np
from live_clearance_core import assess, cloud_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud", default="/cloud_registered")
    parser.add_argument("--odom", default="/ekf_quat/ekf_odom")
    parser.add_argument("--frame", default="world")
    parser.add_argument("--clearance", type=float, default=0.22)
    parser.add_argument("--duration", type=float, default=300)
    args = parser.parse_args()
    if not 0 < args.clearance < 2 or not 0 < args.duration <= 3600:
        parser.error("invalid clearance/duration")
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    rospy.init_node("live_clearance_shadow", anonymous=True, disable_signals=False)
    latest, lock = {}, threading.Lock()

    def receive(key, msg):
        with lock:
            latest[key] = (msg, time.monotonic())

    subscribers = [
        rospy.Subscriber(args.cloud, PointCloud2, lambda m: receive("cloud", m),
                         queue_size=1, buff_size=16 * 1024 * 1024, tcp_nodelay=True),
        rospy.Subscriber(args.odom, Odometry, lambda m: receive("odom", m),
                         queue_size=1, tcp_nodelay=True)]
    start, cpu_start = time.monotonic(), time.process_time()
    timings, delays, counts = [], [], {}
    previous_cloud_stamp = None
    print(json.dumps(dict(mode="SHADOW_ONLY_NO_CONTROL", cloud=args.cloud,
                          frame=args.frame, clearance=args.clearance)), flush=True)
    while not rospy.is_shutdown() and time.monotonic() - start < args.duration:
        tick = time.monotonic()
        try:
            with lock:
                snapshot = latest.copy()
            cloud, cr = snapshot["cloud"]
            odom, od = snapshot["odom"]
            now = rospy.Time.now().to_sec()
            ca, oa = now - cloud.header.stamp.to_sec(), now - odom.header.stamp.to_sec()
            if (cloud.header.frame_id != args.frame or odom.header.frame_id != args.frame):
                raise ValueError("frame mismatch; no implicit coordinate transform")
            if not (-0.02 <= ca <= 0.3 and -0.02 <= oa <= 0.15
                    and tick - cr <= 0.3 and tick - od <= 0.15):
                raise ValueError("stale or future cloud/odometry")
            stamp = cloud.header.stamp.to_nsec()
            if previous_cloud_stamp is not None and stamp < previous_cloud_stamp:
                raise ValueError("cloud time moved backwards")
            previous_cloud_stamp = stamp
            pos, vel = odom.pose.pose.position, odom.twist.twist.linear
            t0 = time.perf_counter()
            result = assess(cloud_xyz(cloud), [pos.x, pos.y, pos.z],
                            [vel.x, vel.y, vel.z], clearance=args.clearance,
                            reaction=0.30 + max(0, ca))
            ms = (time.perf_counter() - t0) * 1000
            timings.append(ms)
            delays.append(oa * 1000)
            result.update(compute_ms=round(ms, 3), odom_age_ms=round(oa * 1000, 2),
                          cloud_age_ms=round(ca * 1000, 2))
        except (KeyError, ValueError, TypeError) as error:
            result = dict(status="INPUT_UNAVAILABLE", reason=str(error))
        counts[result["status"]] = counts.get(result["status"], 0) + 1
        print(json.dumps(result), flush=True)
        time.sleep(max(0, 0.1 - (time.monotonic() - tick)))
    elapsed = time.monotonic() - start
    print(json.dumps(dict(summary=True, seconds=elapsed, counts=counts,
        cpu_one_core_percent=100 * (time.process_time() - cpu_start) / elapsed,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        compute_p95_ms=float(np.percentile(timings, 95)) if timings else None,
        odom_age_p95_ms=float(np.percentile(delays, 95)) if delays else None)), flush=True)
    for sub in subscribers:
        sub.unregister()


if __name__ == "__main__":
    main()
