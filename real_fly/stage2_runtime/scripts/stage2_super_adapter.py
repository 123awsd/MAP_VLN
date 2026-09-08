#!/usr/bin/env python3
"""Guarded Stage-2 mission adapter for the senior SUPER planner.

Preview mode never constructs a goal publisher. Execute mode publishes only
PoseStamped goals; it never publishes PositionCommand, arms, takes off or lands.
"""
import argparse
import hashlib
import json
import math
import sys
import threading
import time
from pathlib import Path

import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path as RosPath
from quadrotor_msgs.msg import Px4ctrlDebug
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class Adapter:
    def __init__(self, args, bundle):
        self.args = args
        self.bundle = bundle
        self.lock = threading.Lock()
        self.odom = None
        self.odom_arrival = 0.0
        self.state = None
        self.px4ctrl_state = None
        self.px4ctrl_arrival = 0.0
        self.abort_reason = None
        self.status_pub = rospy.Publisher("/pre_map_vln/runtime_status", String,
                                         queue_size=1, latch=True)
        self.path_pub = rospy.Publisher("/pre_map_vln/approved_route", RosPath,
                                       queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/pre_map_vln/approved_goals", MarkerArray,
                                         queue_size=1, latch=True)
        self.pause_pub = rospy.Publisher("/planning/super_pause", Bool,
                                        queue_size=1, latch=True)
        # The safety property is structural: preview mode has no goal publisher.
        self.goal_pub = None
        if args.mode == "execute":
            self.goal_pub = rospy.Publisher("/planning/click_goal", PoseStamped,
                                            queue_size=1)
            rospy.on_shutdown(self.pause_super)
        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        rospy.Subscriber("/debugPx4ctrl", Px4ctrlDebug, self.px4ctrl_cb, queue_size=10)
        self.publish_preview()

    def pause_super(self):
        """Best-effort stop request before ROS tears down an execute adapter."""
        try:
            self.pause_pub.publish(Bool(data=True))
        except Exception:
            pass

    def odom_cb(self, message):
        with self.lock:
            self.odom = message
            self.odom_arrival = time.monotonic()

    def state_cb(self, message):
        with self.lock:
            self.state = message

    def px4ctrl_cb(self, message):
        with self.lock:
            self.px4ctrl_state = int(message.state)
            self.px4ctrl_arrival = time.monotonic()

    def status(self, value):
        self.status_pub.publish(String(data=value))
        rospy.loginfo("[stage2_adapter] %s", value)

    def publish_preview(self):
        now = rospy.Time.now()
        path = RosPath()
        path.header.frame_id = "world"
        path.header.stamp = now
        start = self.bundle["start_xyz_yaw"]
        all_points = [start[:3]] + [goal["xyz"] for goal in self.bundle["goals"]]
        for xyz in all_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "world"
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, goal in enumerate(self.bundle["goals"]):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = now
            marker.ns = "approved_execution_goals"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = goal["xyz"]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.10 if goal["kind"] == "transit" else 0.22
            marker.color.r = 0.15 if goal["kind"] == "transit" else 0.10
            marker.color.g = 0.65 if goal["kind"] == "transit" else 0.95
            marker.color.b = 1.00 if goal["kind"] == "transit" else 0.20
            marker.color.a = 0.8
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def snapshot(self):
        with self.lock:
            return (self.odom, self.odom_arrival, self.state,
                    self.px4ctrl_state, self.px4ctrl_arrival)

    def healthy(self, require_armed):
        odom, arrival, state, px4ctrl_state, px4ctrl_arrival = self.snapshot()
        if odom is None or time.monotonic() - arrival > self.args.odom_timeout:
            return False, "odometry missing or stale"
        if odom.header.frame_id != "world":
            return False, f"odometry frame is {odom.header.frame_id!r}, expected 'world'"
        values = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                  odom.pose.pose.position.z, odom.twist.twist.linear.x,
                  odom.twist.twist.linear.y, odom.twist.twist.linear.z]
        if not all(math.isfinite(value) for value in values):
            return False, "odometry contains non-finite values"
        if state is None or not state.connected:
            return False, "flight controller is not connected"
        if require_armed and not state.armed:
            return False, "flight controller is not armed"
        if require_armed:
            if time.monotonic() - px4ctrl_arrival > self.args.px4ctrl_timeout:
                return False, "PX4Ctrl state is missing or stale"
            # 2=AUTO_HOVER and 3=CMD_CTRL. Never release a goal while the
            # controller is manual, taking off, or landing.
            if px4ctrl_state not in (2, 3):
                return False, f"PX4Ctrl is not AUTO_HOVER/CMD_CTRL (state={px4ctrl_state})"
        return True, "ok"

    @staticmethod
    def position_and_speed(odom):
        position = odom.pose.pose.position
        velocity = odom.twist.twist.linear
        return ([position.x, position.y, position.z],
                math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2))

    def wait_ready(self, require_armed, timeout):
        deadline = time.monotonic() + timeout
        reason = "waiting"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            ok, reason = self.healthy(require_armed)
            if ok:
                return
            rospy.sleep(0.1)
        raise RuntimeError(reason)

    def run_preview(self):
        self.status("PREVIEW_ONLY: no planner goal publisher exists")
        rospy.spin()

    def run_execute(self):
        if not self.args.acknowledge_motion_only:
            raise RuntimeError(
                "online target verification is not connected; execute mode requires "
                "--acknowledge-motion-only for a restrained motion test"
            )
        if not self.bundle.get("safety", {}).get("start_pose_explicitly_approved", False):
            raise RuntimeError(
                "mission start came from an automatic preview pose; regenerate with an "
                "explicit approved --start pose"
            )
        if self.args.confirm_execute != self.bundle["map_sha256"]:
            raise RuntimeError("--confirm-execute must equal the complete approved map SHA256")
        self.wait_ready(require_armed=True, timeout=self.args.ready_timeout)
        if self.goal_pub.get_num_connections() < 1:
            raise RuntimeError("SUPER is not subscribed to /planning/click_goal")
        master = rosgraph.Master(rospy.get_name())
        publishers, subscribers, _ = master.getSystemState()
        publishers = {topic: nodes for topic, nodes in publishers}
        subscribers = {topic: nodes for topic, nodes in subscribers}
        if not publishers.get("/planning/pos_cmd"):
            raise RuntimeError("no command mux publishes /planning/pos_cmd")
        if not subscribers.get("/planning/pos_cmd"):
            raise RuntimeError("PX4Ctrl is not subscribed to /planning/pos_cmd")
        if not subscribers.get("/planning/super_pos_cmd"):
            raise RuntimeError("command mux is not subscribed to /planning/super_pos_cmd")
        odom, _, _, _, _ = self.snapshot()
        position, speed = self.position_and_speed(odom)
        start_error = math.dist(position, self.bundle["start_xyz_yaw"][:3])
        if start_error > self.args.start_tolerance:
            raise RuntimeError(f"start mismatch {start_error:.3f} m > {self.args.start_tolerance:.3f} m")
        if speed > self.args.start_speed:
            raise RuntimeError(f"vehicle speed {speed:.3f} m/s exceeds start gate")
        self.pause_pub.publish(Bool(data=False))
        for index, goal in enumerate(self.bundle["goals"]):
            ok, reason = self.healthy(require_armed=True)
            if not ok:
                raise RuntimeError(reason)
            goal_kind = goal.get("kind")
            if goal_kind not in ("transit", "observation"):
                raise RuntimeError(f"unsupported goal kind: {goal_kind!r}")
            message = PoseStamped()
            message.header.frame_id = "world"
            message.header.stamp = rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = goal["xyz"]
            if goal["yaw"] is None:
                message.pose.orientation.x = float("nan")
                message.pose.orientation.y = float("nan")
                message.pose.orientation.z = float("nan")
                message.pose.orientation.w = float("nan")
            else:
                qx, qy, qz, qw = yaw_quaternion(goal["yaw"])
                message.pose.orientation.x, message.pose.orientation.y = qx, qy
                message.pose.orientation.z, message.pose.orientation.w = qz, qw
            self.goal_pub.publish(message)
            self.status(f"EXECUTING goal {index + 1}/{len(self.bundle['goals'])}: {goal_kind}")
            deadline = time.monotonic() + self.args.goal_timeout
            dwell_started = None
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                ok, reason = self.healthy(require_armed=True)
                if not ok:
                    raise RuntimeError(reason)
                odom, _, _, _, _ = self.snapshot()
                position, speed = self.position_and_speed(odom)
                distance = math.dist(position, goal["xyz"])
                if goal_kind == "transit" and distance <= self.args.transit_switch_radius:
                    self.status(
                        f"PASSING goal {index + 1}/{len(self.bundle['goals'])}: "
                        f"switching at {distance:.2f} m without stopping"
                    )
                    break
                if (goal_kind == "observation" and
                        distance <= self.args.goal_tolerance and
                        speed <= self.args.arrival_speed):
                    dwell_started = dwell_started or time.monotonic()
                    if time.monotonic() - dwell_started >= self.args.arrival_dwell:
                        break
                else:
                    dwell_started = None
                rospy.sleep(0.05)
            else:
                raise RuntimeError(f"goal {index + 1} timed out")
        self.pause_pub.publish(Bool(data=True))
        self.status("COMPLETE: goals reached; SUPER paused; no landing command sent")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--map", dest="map_pcd", type=Path, required=True)
    parser.add_argument("--mode", choices=("preview", "execute"), default="preview")
    parser.add_argument("--confirm-execute", default="")
    parser.add_argument("--acknowledge-motion-only", action="store_true")
    parser.add_argument("--odom-topic", default="/ekf_quat/ekf_odom")
    parser.add_argument("--odom-timeout", type=float, default=0.25)
    parser.add_argument("--px4ctrl-timeout", type=float, default=0.25)
    parser.add_argument("--ready-timeout", type=float, default=15.0)
    parser.add_argument("--start-tolerance", type=float, default=0.35)
    parser.add_argument("--start-speed", type=float, default=0.20)
    parser.add_argument("--transit-switch-radius", type=float, default=0.40)
    parser.add_argument("--goal-tolerance", type=float, default=0.20)
    parser.add_argument("--arrival-speed", type=float, default=0.20)
    parser.add_argument("--arrival-dwell", type=float, default=0.75)
    parser.add_argument("--goal-timeout", type=float, default=45.0)
    args = parser.parse_args()
    if not 0.10 <= args.transit_switch_radius <= 1.0:
        raise SystemExit("--transit-switch-radius must be in [0.10, 1.0] m")
    if not 0.05 <= args.goal_tolerance <= 1.0:
        raise SystemExit("--goal-tolerance must be in [0.05, 1.0] m")
    bundle = json.loads(args.bundle.read_text(encoding="utf-8"))
    if bundle.get("format") != "pre_map_vln.real_execution_bundle.v1":
        raise SystemExit("unsupported execution bundle")
    actual_hash = sha256(args.map_pcd)
    if actual_hash != bundle.get("map_sha256"):
        raise SystemExit(f"map SHA256 mismatch: expected {bundle.get('map_sha256')}, got {actual_hash}")
    rospy.init_node("pre_map_vln_stage2_super_adapter", anonymous=False)
    adapter = Adapter(args, bundle)
    try:
        adapter.run_preview() if args.mode == "preview" else adapter.run_execute()
    except Exception as error:
        adapter.pause_pub.publish(Bool(data=True))
        adapter.status(f"ABORTED: {error}")
        rospy.logerr("[stage2_adapter] aborted: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
