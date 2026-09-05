#!/usr/bin/env python3
"""Read-only rolling health display for the Stage-2 localization topics."""

import collections
import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg


WINDOW_SEC = 5.0
STALE_SEC = 1.0


class TopicHealth:
    def __init__(self, expected_hz):
        self.expected_hz = expected_hz
        self.arrivals = collections.deque()
        self.last_arrival = None
        self.last_stamp = None

    def update(self, msg):
        now = time.monotonic()
        self.arrivals.append(now)
        self.last_arrival = now
        self.last_stamp = msg.header.stamp.to_sec()
        cutoff = now - WINDOW_SEC
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()

    def rate(self, now):
        cutoff = now - WINDOW_SEC
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()
        if len(self.arrivals) < 2:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0.0 else 0.0

    def freshness(self, now):
        return math.inf if self.last_arrival is None else now - self.last_arrival


class LocalizationMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.health = {
            "LiDAR": TopicHealth(20.0),
            "FCU IMU": TopicHealth(200.0),
            "Odometry": TopicHealth(20.0),
        }
        self.odom_delay_ms = None
        self.max_odom_delay_ms = None
        self.delay_samples = collections.deque()

        rospy.Subscriber("/livox/lidar", CustomMsg, self.lidar_cb, queue_size=20)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb, queue_size=400)
        rospy.Subscriber("/Odometry", Odometry, self.odom_cb, queue_size=50)

    def lidar_cb(self, msg):
        with self.lock:
            self.health["LiDAR"].update(msg)

    def imu_cb(self, msg):
        with self.lock:
            self.health["FCU IMU"].update(msg)

    def odom_cb(self, msg):
        wall_now = rospy.Time.now().to_sec()
        delay_ms = max(0.0, (wall_now - msg.header.stamp.to_sec()) * 1000.0)
        monotonic_now = time.monotonic()
        with self.lock:
            self.health["Odometry"].update(msg)
            self.odom_delay_ms = delay_ms
            self.delay_samples.append((monotonic_now, delay_ms))
            cutoff = monotonic_now - WINDOW_SEC
            while self.delay_samples and self.delay_samples[0][0] < cutoff:
                self.delay_samples.popleft()
            self.max_odom_delay_ms = max(value for _, value in self.delay_samples)

    def display(self):
        now = time.monotonic()
        lines = []
        overall_ok = True
        with self.lock:
            for name, health in self.health.items():
                hz = health.rate(now)
                age = health.freshness(now)
                if age > STALE_SEC:
                    state = "WAIT" if math.isinf(age) else "STALE"
                    overall_ok = False
                elif hz < health.expected_hz * 0.80:
                    state = "LOW"
                    overall_ok = False
                else:
                    state = "OK"
                age_text = "--" if math.isinf(age) else f"{age * 1000.0:.0f} ms"
                lines.append(
                    f"{name:<10} {hz:7.1f} Hz  age {age_text:>8}  [{state}]"
                )

            delay = self.odom_delay_ms
            delay_max = self.max_odom_delay_ms

        if delay is None:
            delay_line = "Odom delay       --"
            overall_ok = False
        else:
            delay_state = "OK" if delay < 100.0 and (delay_max or 0.0) < 150.0 else "HIGH"
            if delay_state != "OK":
                overall_ok = False
            delay_line = (
                f"Odom delay {delay:8.1f} ms  rolling max {delay_max:8.1f} ms  "
                f"[{delay_state}]"
            )

        print("\033[H\033[2J", end="")
        print("====== Stage-2 localization health (read only) ======")
        print("\n".join(lines))
        print(delay_line)
        print("-----------------------------------------------------")
        print("Overall: " + ("OK" if overall_ok else "NOT READY"))
        print("No control topic is published. Ctrl-C exits monitor.")


def main():
    rospy.init_node("localization_health_monitor", anonymous=False)
    monitor = LocalizationMonitor()
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        monitor.display()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
