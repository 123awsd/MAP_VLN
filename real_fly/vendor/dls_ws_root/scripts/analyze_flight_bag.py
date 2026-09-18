#!/usr/bin/env python3
"""Analyze a SUPER + PX4Ctrl ROS1 flight bag without replaying it."""

import argparse
import csv
import json
import math
import os
import re
import shlex
import sys
from collections import Counter
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent

# Direct execution is convenient on the aircraft. Re-exec once through bash so
# ROS Python modules, LZ4 libraries, and workspace custom messages are all set.
ros_setup = Path("/opt/ros/noetic/setup.bash")
workspace_setup = WORKSPACE / "devel/setup.bash"
if (
    not os.environ.get("SUPER_BAG_ANALYSIS_BOOTSTRAPPED")
    and ros_setup.is_file()
    and workspace_setup.is_file()
):
    command = "source {} && source {} && export SUPER_BAG_ANALYSIS_BOOTSTRAPPED=1 && exec {}".format(
        shlex.quote(str(ros_setup)),
        shlex.quote(str(workspace_setup)),
        shlex.join([sys.executable] + sys.argv),
    )
    os.execv("/bin/bash", ["/bin/bash", "-lc", command])

try:
    import rosbag
except ImportError as exc:
    print(
        "错误：无法导入 rosbag。请先执行 source /opt/ros/noetic/setup.bash 和 "
        "source devel/setup.bash，再运行本脚本。",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc

try:
    import numpy as np
except ImportError as exc:
    print("错误：缺少 numpy，请执行 pip3 install numpy。", file=sys.stderr)
    raise SystemExit(1) from exc

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    print("错误：缺少 matplotlib，请执行 pip3 install matplotlib。", file=sys.stderr)
    raise SystemExit(1) from exc


ODOM_CANDIDATES = ("/ekf_quat/ekf_odom", "/Odometry", "/mavros/local_position/odom")
CMD_CANDIDATES = ("/planning/pos_cmd",)
SUPER_CMD_TOPIC = "/planning/super_pos_cmd"
GATE_CMD_TOPIC = "/planning/gate_pos_cmd"
GATE_ACTIVE_TOPIC = "/planning/gate_active"
SUPER_PAUSE_TOPIC = "/planning/super_pause"
GATE_MARKERS_TOPIC = "/planning/d_gate_smooth_markers"
DEBUG_TOPIC = "/debugPx4ctrl"
BATTERY_TOPIC = "/mavros/battery"
IMU_TOPIC = "/mavros/imu/data"
ATTITUDE_TARGET_TOPIC = "/mavros/setpoint_raw/attitude"
STATE_TOPIC = "/mavros/state"
EXTENDED_STATE_TOPIC = "/mavros/extended_state"
GOAL_TOPIC = "/planning/click_goal"
TRIGGER_TOPIC = "/traj_start_trigger"
TAKEOFF_LAND_TOPIC = "/px4ctrl/takeoff_land"
ROSOUT_TOPIC = "/rosout"


def parse_args():
    parser = argparse.ArgumentParser(
        description="分析 SUPER 规划指令、EKF 里程计、PX4Ctrl 和 MAVROS 飞行数据。"
    )
    parser.add_argument(
        "bags",
        nargs="*",
        help="bag 文件或包含 bag 的目录；不指定时自动使用 flight_bags 中最新文件",
    )
    parser.add_argument("--output", help="分析结果目录")
    parser.add_argument("--odom-topic", help="实际里程计 topic")
    parser.add_argument("--cmd-topic", help="SUPER PositionCommand topic")
    parser.add_argument(
        "--sync-max-gap",
        type=float,
        default=0.05,
        help="里程计与最近指令允许的最大接收时间差，默认 0.05 s",
    )
    parser.add_argument(
        "--speed-limit",
        type=float,
        default=2.5,
        help="报告和图中使用的比赛速度上限，默认 2.5 m/s",
    )
    return parser.parse_args()


def resolve_bags(raw_paths, workspace):
    candidates = []
    if raw_paths:
        for raw in raw_paths:
            path = Path(raw).expanduser().resolve()
            if path.is_dir():
                directory_bags = list(path.glob("*.bag"))
                if directory_bags:
                    candidates.append(max(directory_bags, key=lambda item: item.stat().st_mtime))
            elif path.is_file() and path.suffix == ".bag":
                candidates.append(path)
            else:
                raise FileNotFoundError("找不到 bag 文件或目录：{}".format(path))
    else:
        bag_root = workspace / "flight_bags"
        candidates = list(bag_root.glob("*.bag")) if bag_root.exists() else []
        if candidates:
            candidates = [max(candidates, key=lambda item: item.stat().st_mtime)]

    unique = []
    seen = set()
    for path in sorted(candidates):
        if path not in seen and not str(path).endswith(".active"):
            unique.append(path)
            seen.add(path)
    if not unique:
        raise FileNotFoundError("没有找到可分析的 .bag 文件。")
    return unique


def inspect_bags(paths):
    available = set()
    counts = Counter()
    start = math.inf
    end = -math.inf
    for path in paths:
        with rosbag.Bag(str(path), "r") as bag:
            start = min(start, bag.get_start_time())
            end = max(end, bag.get_end_time())
            topic_info = bag.get_type_and_topic_info()[1]
            for topic, info in topic_info.items():
                available.add(topic)
                counts[topic] += info.message_count
    return available, counts, start, end


def select_topic(explicit, candidates, available, label, required=True):
    if explicit:
        if explicit not in available:
            raise RuntimeError("{} topic 不在 bag 中：{}".format(label, explicit))
        return explicit
    for candidate in candidates:
        if candidate in available:
            return candidate
    if required:
        raise RuntimeError("bag 中找不到{}，候选为：{}".format(label, ", ".join(candidates)))
    return None


def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_angle(values):
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def as_array(values, width=None):
    if not values:
        if width is None:
            return np.empty((0,), dtype=float)
        return np.empty((0, width), dtype=float)
    return np.asarray(values, dtype=float)


def field(msg, name, default=float("nan")):
    return float(getattr(msg, name, default))


def message_time(msg, bag_time):
    """Prefer the publisher timestamp and fall back to rosbag receive time."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        timestamp = stamp.to_sec()
        if timestamp > 0.0:
            return timestamp
    return bag_time.to_sec()


def nearest_time_distance(reference_times, query_times):
    indices = np.searchsorted(reference_times, query_times)
    right = np.clip(indices, 0, len(reference_times) - 1)
    left = np.clip(indices - 1, 0, len(reference_times) - 1)
    return np.minimum(
        np.abs(query_times - reference_times[left]),
        np.abs(query_times - reference_times[right]),
    )


def interpolate_columns(source_t, source_values, query_t):
    if source_values.ndim == 1:
        return np.interp(query_t, source_t, source_values)
    return np.column_stack(
        [np.interp(query_t, source_t, source_values[:, index]) for index in range(source_values.shape[1])]
    )


def scalar_stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def norm_stats(values):
    if values.size == 0:
        return None
    return scalar_stats(np.linalg.norm(values, axis=1))


def frequency(times):
    if len(times) < 2:
        return None
    duration = times[-1] - times[0]
    return float((len(times) - 1) / duration) if duration > 0 else None


def transition_list(samples, key):
    transitions = []
    previous = object()
    for sample in samples:
        current = sample[key]
        if current != previous:
            transitions.append(sample)
            previous = current
    return transitions


def fmt(value, digits=3, suffix=""):
    if value is None or not math.isfinite(float(value)):
        return "N/A"
    return ("{:,.%df}%s" % (digits, suffix)).format(float(value))


def empty_command_group():
    return {
        "t": [], "p": [], "v": [], "a": [], "j": [], "yaw": [],
        "yaw_rate": [], "flag": [], "frames": set(),
    }


def append_command(group, msg, timestamp):
    group["t"].append(timestamp)
    group["p"].append([msg.position.x, msg.position.y, msg.position.z])
    group["v"].append([msg.velocity.x, msg.velocity.y, msg.velocity.z])
    group["a"].append([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z])
    group["j"].append([msg.jerk.x, msg.jerk.y, msg.jerk.z])
    group["yaw"].append(float(msg.yaw))
    group["yaw_rate"].append(float(msg.yaw_dot))
    group["flag"].append(int(msg.trajectory_flag))
    group["frames"].add(msg.header.frame_id or "<empty>")


def read_data(paths, topics):
    data = {
        "odom": {"t": [], "p": [], "v": [], "yaw": [], "frames": set()},
        "cmd": empty_command_group(),
        "super_cmd": empty_command_group(),
        "gate_cmd": empty_command_group(),
        "battery": {"t": [], "voltage": [], "current": [], "percentage": []},
        "debug": {"t": [], "att_err": [], "des_thr": [], "ideal_thr": [], "hover": [], "voltage": [], "fb_rate": [], "ideal_rate": []},
        "imu": {"t": [], "acc": [], "rate": []},
        "attitude_target": {"t": [], "thrust": []},
        "states": [],
        "landed_states": [],
        "goals": [],
        "triggers": [],
        "takeoff_land": [],
        "gate_active": [],
        "super_pause": [],
        "gate_safety": [],
        "rosout": [],
    }

    read_topics = {topic for topic in topics.values() if topic}
    for path in paths:
        try:
            bag = rosbag.Bag(str(path), "r")
        except Exception as exc:
            raise RuntimeError("无法打开 {}：{}".format(path, exc)) from exc
        with bag:
            try:
                iterator = bag.read_messages(topics=sorted(read_topics))
                for topic, msg, bag_time in iterator:
                    timestamp = message_time(msg, bag_time)
                    if topic == topics["odom"]:
                        data["odom"]["t"].append(timestamp)
                        data["odom"]["p"].append(
                            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
                        )
                        data["odom"]["v"].append(
                            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
                        )
                        data["odom"]["yaw"].append(quaternion_to_yaw(msg.pose.pose.orientation))
                        data["odom"]["frames"].add(msg.header.frame_id or "<empty>")
                    elif topic == topics["cmd"]:
                        append_command(data["cmd"], msg, timestamp)
                    elif topics.get("super_cmd") and topic == topics["super_cmd"]:
                        append_command(data["super_cmd"], msg, timestamp)
                    elif topics.get("gate_cmd") and topic == topics["gate_cmd"]:
                        append_command(data["gate_cmd"], msg, timestamp)
                    elif topic == BATTERY_TOPIC:
                        data["battery"]["t"].append(timestamp)
                        data["battery"]["voltage"].append(field(msg, "voltage"))
                        data["battery"]["current"].append(field(msg, "current"))
                        data["battery"]["percentage"].append(field(msg, "percentage"))
                    elif topic == DEBUG_TOPIC:
                        debug = data["debug"]
                        debug["t"].append(timestamp)
                        debug["att_err"].append(field(msg, "exec_err_axisang_ang"))
                        debug["des_thr"].append(field(msg, "des_thr"))
                        debug["ideal_thr"].append(field(msg, "ideal_thr"))
                        debug["hover"].append(field(msg, "hover_percentage"))
                        debug["voltage"].append(field(msg, "voltage"))
                        debug["fb_rate"].append([field(msg, "fb_rate_x"), field(msg, "fb_rate_y"), field(msg, "fb_rate_z")])
                        debug["ideal_rate"].append([field(msg, "ideal_rate_x"), field(msg, "ideal_rate_y"), field(msg, "ideal_rate_z")])
                    elif topic == IMU_TOPIC:
                        data["imu"]["t"].append(timestamp)
                        data["imu"]["acc"].append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                        data["imu"]["rate"].append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                    elif topic == ATTITUDE_TARGET_TOPIC:
                        data["attitude_target"]["t"].append(timestamp)
                        data["attitude_target"]["thrust"].append(float(msg.thrust))
                    elif topic == STATE_TOPIC:
                        data["states"].append(
                            {"t": timestamp, "connected": bool(msg.connected), "armed": bool(msg.armed), "mode": msg.mode}
                        )
                    elif topic == EXTENDED_STATE_TOPIC:
                        data["landed_states"].append({"t": timestamp, "state": int(msg.landed_state)})
                    elif topic == GOAL_TOPIC:
                        data["goals"].append(
                            {"t": timestamp, "position": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]}
                        )
                    elif topic == TRIGGER_TOPIC:
                        data["triggers"].append(timestamp)
                    elif topic == TAKEOFF_LAND_TOPIC:
                        data["takeoff_land"].append({"t": timestamp, "command": int(msg.takeoff_land_cmd)})
                    elif topics.get("gate_active") and topic == topics["gate_active"]:
                        data["gate_active"].append({"t": timestamp, "value": bool(msg.data)})
                    elif topics.get("super_pause") and topic == topics["super_pause"]:
                        data["super_pause"].append({"t": timestamp, "value": bool(msg.data)})
                    elif topics.get("gate_markers") and topic == topics["gate_markers"]:
                        for marker in msg.markers:
                            marker_text = str(marker.text).strip()
                            if not marker_text or "smooth" not in marker_text.lower():
                                continue
                            match = re.search(r"min\s*[=<]\s*([0-9]+(?:\.[0-9]+)?)", marker_text)
                            status = "unknown"
                            for candidate in ("safe", "collision", "waiting", "blocked"):
                                if candidate in marker_text.lower():
                                    status = candidate
                                    break
                            data["gate_safety"].append(
                                {
                                    "t": timestamp,
                                    "status": status,
                                    "minimum_clearance_m": float(match.group(1)) if match else None,
                                    "text": marker_text,
                                }
                            )
                    elif topic == ROSOUT_TOPIC:
                        message = str(msg.msg)
                        relevant = any(token in message for token in ("[Fsm]", "[MISSION]", "[SUPER]", "[ROG", "[PX4Ctrl]"))
                        if relevant or int(msg.level) >= 4:
                            data["rosout"].append(
                                {"t": timestamp, "level": int(msg.level), "node": str(msg.name), "message": message}
                            )
            except Exception as exc:
                raise RuntimeError(
                    "读取 {} 失败：{}。请确认已 source 当前工作区，使自定义消息可被 rosbag 解析。".format(path, exc)
                ) from exc
    return data


def convert_arrays(data):
    for group in (
        "odom", "cmd", "super_cmd", "gate_cmd", "battery", "debug", "imu",
        "attitude_target",
    ):
        for key, values in list(data[group].items()):
            if key == "frames":
                data[group][key] = sorted(values)
            elif key in ("p", "v", "a", "j", "fb_rate", "ideal_rate", "acc", "rate"):
                data[group][key] = as_array(values, 3)
            else:
                data[group][key] = as_array(values)
    return data


def compute_tracking(data, max_gap):
    odom = data["odom"]
    cmd = data["cmd"]
    if len(odom["t"]) == 0 or len(cmd["t"]) == 0:
        return None

    # np.interp requires strictly increasing source times. Keep the final command at duplicate times.
    reverse_unique = np.unique(cmd["t"][::-1], return_index=True)[1]
    cmd_indices = np.sort(len(cmd["t"]) - 1 - reverse_unique)
    cmd_t = cmd["t"][cmd_indices]
    cmd_p = cmd["p"][cmd_indices]
    cmd_v = cmd["v"][cmd_indices]
    cmd_yaw = cmd["yaw"][cmd_indices]

    overlap = (odom["t"] >= cmd_t[0]) & (odom["t"] <= cmd_t[-1])
    odom_indices = np.flatnonzero(overlap)
    if odom_indices.size == 0:
        return None
    nearest_gap = nearest_time_distance(cmd_t, odom["t"][odom_indices])
    odom_indices = odom_indices[nearest_gap <= max_gap]
    if odom_indices.size == 0:
        return None

    query_t = odom["t"][odom_indices]
    desired_p = interpolate_columns(cmd_t, cmd_p, query_t)
    desired_v = interpolate_columns(cmd_t, cmd_v, query_t)
    desired_yaw = interpolate_columns(cmd_t, np.unwrap(cmd_yaw), query_t)
    position_error = odom["p"][odom_indices] - desired_p
    velocity_error = odom["v"][odom_indices] - desired_v
    yaw_error = wrap_angle(odom["yaw"][odom_indices] - desired_yaw)

    return {
        "t": query_t,
        "odom_indices": odom_indices,
        "desired_p": desired_p,
        "desired_v": desired_v,
        "desired_yaw": desired_yaw,
        "position_error": position_error,
        "velocity_error": velocity_error,
        "yaw_error": yaw_error,
    }


def true_intervals(samples, end_time):
    intervals = []
    active_start = None
    for sample in sorted(samples, key=lambda item: item["t"]):
        if sample["value"] and active_start is None:
            active_start = sample["t"]
        elif not sample["value"] and active_start is not None:
            if sample["t"] > active_start:
                intervals.append((active_start, sample["t"]))
            active_start = None
    if active_start is not None and end_time > active_start:
        intervals.append((active_start, end_time))
    return intervals


def interval_mask(times, intervals):
    mask = np.zeros(len(times), dtype=bool)
    for begin, end in intervals:
        mask |= (times >= begin) & (times <= end)
    return mask


def summarize_tracking(tracking, odom, speed_limit, mask=None):
    if not tracking:
        return None
    if mask is None:
        mask = np.ones(len(tracking["t"]), dtype=bool)
    if not np.any(mask):
        return None
    times = tracking["t"][mask]
    position_error = tracking["position_error"][mask]
    velocity_error = tracking["velocity_error"][mask]
    yaw_error = tracking["yaw_error"][mask]
    odom_indices = tracking["odom_indices"][mask]
    tracked_speed = np.linalg.norm(odom["v"][odom_indices], axis=1)
    return {
        "samples": int(len(times)),
        "duration_s": float(times[-1] - times[0]) if len(times) > 1 else 0.0,
        "position_error_m": scalar_stats(np.linalg.norm(position_error, axis=1)),
        "position_axis_rmse_m": [
            float(value) for value in np.sqrt(np.mean(position_error ** 2, axis=0))
        ],
        "velocity_error_mps": scalar_stats(np.linalg.norm(velocity_error, axis=1)),
        "yaw_error_deg": scalar_stats(np.abs(np.rad2deg(yaw_error))),
        "actual_speed_mps": scalar_stats(tracked_speed),
        "actual_samples_over_speed_limit": int(np.sum(tracked_speed > speed_limit)),
    }


def summarize_command(group, speed_limit, intervals=None):
    if len(group["t"]) == 0:
        return None
    mask = (
        interval_mask(group["t"], intervals)
        if intervals is not None
        else np.ones(len(group["t"]), dtype=bool)
    )
    if not np.any(mask):
        return None
    times = group["t"][mask]
    speed = np.linalg.norm(group["v"][mask], axis=1)
    acceleration = np.linalg.norm(group["a"][mask], axis=1)
    jerk = np.linalg.norm(group["j"][mask], axis=1)
    return {
        "samples": int(len(times)),
        "duration_s": float(times[-1] - times[0]) if len(times) > 1 else 0.0,
        "speed_mps": scalar_stats(speed),
        "acceleration_mps2": scalar_stats(acceleration),
        "jerk_mps3": scalar_stats(jerk),
        "samples_over_speed_limit": int(np.sum(speed > speed_limit)),
    }


def summarize_handoffs(super_cmd, gate_cmd, intervals, maximum_gap=0.35):
    handoffs = []
    if len(super_cmd["t"]) == 0 or len(gate_cmd["t"]) == 0:
        return None
    for begin, _ in intervals:
        super_index = int(np.searchsorted(super_cmd["t"], begin, side="right") - 1)
        gate_index = int(np.searchsorted(gate_cmd["t"], begin, side="left"))
        if super_index < 0 or gate_index >= len(gate_cmd["t"]):
            continue
        before_gap = begin - super_cmd["t"][super_index]
        after_gap = gate_cmd["t"][gate_index] - begin
        if before_gap > maximum_gap or after_gap > maximum_gap:
            continue
        handoffs.append(
            {
                "t": float(begin),
                "super_age_s": float(before_gap),
                "gate_delay_s": float(after_gap),
                "position_jump_m": float(np.linalg.norm(
                    gate_cmd["p"][gate_index] - super_cmd["p"][super_index]
                )),
                "velocity_jump_mps": float(np.linalg.norm(
                    gate_cmd["v"][gate_index] - super_cmd["v"][super_index]
                )),
                "acceleration_jump_mps2": float(np.linalg.norm(
                    gate_cmd["a"][gate_index] - super_cmd["a"][super_index]
                )),
            }
        )
    if not handoffs:
        return None
    return {
        "count": len(handoffs),
        "max_position_jump_m": max(item["position_jump_m"] for item in handoffs),
        "max_velocity_jump_mps": max(item["velocity_jump_mps"] for item in handoffs),
        "max_acceleration_jump_mps2": max(item["acceleration_jump_mps2"] for item in handoffs),
        "items": handoffs,
    }


def first_time_after(samples, begin, predicate):
    for sample in samples:
        if sample["t"] >= begin and predicate(sample):
            return sample["t"]
    return None


def build_timing_summary(data, smooth_intervals):
    trigger = data["triggers"][0] if data["triggers"] else None
    takeoff = next((item["t"] for item in data["takeoff_land"] if item["command"] == 1), None)
    land = next((item["t"] for item in data["takeoff_land"] if item["command"] == 2), None)
    first_command = None
    if trigger is not None and len(data["cmd"]["t"]):
        index = int(np.searchsorted(data["cmd"]["t"], trigger, side="left"))
        if index < len(data["cmd"]["t"]):
            first_command = float(data["cmd"]["t"][index])
    on_ground = first_time_after(
        data["landed_states"], land, lambda item: item["state"] == 1
    ) if land is not None else None
    disarmed = first_time_after(
        data["states"], land, lambda item: not item["armed"]
    ) if land is not None else None
    return {
        "takeoff_command_to_trigger_s": (
            float(trigger - takeoff) if takeoff is not None and trigger is not None else None
        ),
        "trigger_to_first_command_s": (
            float(first_command - trigger) if trigger is not None and first_command is not None else None
        ),
        "command_active_span_s": (
            float(data["cmd"]["t"][-1] - data["cmd"]["t"][0])
            if len(data["cmd"]["t"]) > 1 else None
        ),
        "smooth_active_duration_s": float(sum(end - begin for begin, end in smooth_intervals)),
        "land_request_to_on_ground_s": (
            float(on_ground - land) if land is not None and on_ground is not None else None
        ),
        "land_request_to_disarm_s": (
            float(disarmed - land) if land is not None and disarmed is not None else None
        ),
    }


def build_goal_segments(data, smooth_intervals):
    goals = data["goals"]
    if not goals:
        return []
    command_end = float(data["cmd"]["t"][-1]) if len(data["cmd"]["t"]) else None
    land_times = [item["t"] for item in data["takeoff_land"] if item["command"] == 2]
    segments = []
    for index, goal in enumerate(goals):
        candidates = []
        if index + 1 < len(goals):
            candidates.append(goals[index + 1]["t"])
        candidates.extend(begin for begin, _ in smooth_intervals if begin > goal["t"])
        candidates.extend(value for value in land_times if value > goal["t"])
        if command_end is not None and command_end > goal["t"]:
            candidates.append(command_end)
        end = min(candidates) if candidates else None
        segments.append(
            {
                "index": index + 1,
                "publish_time": float(goal["t"]),
                "position": goal["position"],
                "duration_until_next_event_s": float(end - goal["t"]) if end is not None else None,
            }
        )
    return segments


def build_summary(paths, available, topic_counts, start, end, topics, data, tracking, speed_limit):
    odom = data["odom"]
    cmd = data["cmd"]
    cmd_speed = np.linalg.norm(cmd["v"], axis=1) if len(cmd["v"]) else np.empty(0)
    cmd_acc = np.linalg.norm(cmd["a"], axis=1) if len(cmd["a"]) else np.empty(0)
    cmd_jerk = np.linalg.norm(cmd["j"], axis=1) if len(cmd["j"]) else np.empty(0)
    odom_speed = np.linalg.norm(odom["v"], axis=1) if len(odom["v"]) else np.empty(0)
    cmd_gaps = np.diff(cmd["t"]) if len(cmd["t"]) > 1 else np.empty(0)

    planner_failures = [
        item for item in data["rosout"]
        if any(token in item["message"].lower() for token in ("failed", "nan", "inf", "opt_failed"))
        and any(token in item["message"] for token in ("[Fsm]", "[SUPER]", "[ExpOpt]", "[BackOpt]", "[MINCO]"))
    ]
    planner_successes = [
        item for item in data["rosout"]
        if "succeed" in item["message"].lower() or "success" in item["message"].lower()
    ]

    states = transition_list(data["states"], "mode")
    armed = transition_list(data["states"], "armed")
    landed = transition_list(data["landed_states"], "state")

    tracking_summary = summarize_tracking(tracking, odom, speed_limit)

    command_end = float(cmd["t"][-1]) if len(cmd["t"]) else float(end)
    smooth_intervals = true_intervals(data["gate_active"], command_end)
    pause_intervals = true_intervals(data["super_pause"], command_end)
    smooth_tracking = None
    if tracking and smooth_intervals:
        smooth_tracking = summarize_tracking(
            tracking, odom, speed_limit, interval_mask(tracking["t"], smooth_intervals)
        )
    smooth_command = summarize_command(cmd, speed_limit, smooth_intervals)
    source_handoffs = summarize_handoffs(data["super_cmd"], data["gate_cmd"], smooth_intervals)
    if topics.get("gate_active") is None:
        detected_mode = "unknown_old_bag"
    elif smooth_intervals and len(data["super_cmd"]["t"]):
        detected_mode = "hybrid"
    elif smooth_intervals:
        detected_mode = "full_smooth"
    else:
        detected_mode = "super"

    finite_clearances = [
        item["minimum_clearance_m"] for item in data["gate_safety"]
        if item["minimum_clearance_m"] is not None
    ]
    safety_transitions = transition_list(data["gate_safety"], "status")

    debug = data["debug"]
    debug_summary = None
    if len(debug["t"]):
        rate_error = debug["fb_rate"] - debug["ideal_rate"]
        debug_summary = {
            "samples": int(len(debug["t"])),
            "frequency_hz": frequency(debug["t"]),
            "attitude_error_deg": scalar_stats(np.abs(debug["att_err"])),
            "body_rate_error_radps": norm_stats(rate_error),
            "desired_thrust": scalar_stats(debug["des_thr"]),
            "ideal_thrust": scalar_stats(debug["ideal_thr"]),
            "hover_percentage": scalar_stats(debug["hover"]),
        }

    battery_voltage = data["battery"]["voltage"]
    valid_voltage = battery_voltage[np.isfinite(battery_voltage) & (battery_voltage > 1.0)]
    battery_summary = None
    if valid_voltage.size:
        battery_summary = {
            "start_v": float(valid_voltage[0]),
            "end_v": float(valid_voltage[-1]),
            "min_v": float(np.min(valid_voltage)),
            "max_v": float(np.max(valid_voltage)),
        }

    return {
        "bags": [str(path) for path in paths],
        "recording": {
            "start_ros_time": float(start),
            "end_ros_time": float(end),
            "duration_s": float(end - start),
            "topic_count": len(available),
        },
        "selected_topics": topics,
        "selected_topic_message_counts": {
            topic: int(topic_counts[topic]) for topic in topics.values() if topic
        },
        "frames": {"odom": odom["frames"], "command": cmd["frames"]},
        "command": {
            "samples": int(len(cmd["t"])),
            "frequency_hz": frequency(cmd["t"]),
            "active_span_s": float(cmd["t"][-1] - cmd["t"][0]) if len(cmd["t"]) > 1 else 0.0,
            "gap_count_over_0_2s": int(np.sum(cmd_gaps > 0.2)),
            "speed_mps": scalar_stats(cmd_speed),
            "acceleration_mps2": scalar_stats(cmd_acc),
            "jerk_mps3": scalar_stats(cmd_jerk),
            "backup_ratio": float(np.mean(cmd["flag"] == 2)) if len(cmd["flag"]) else None,
            "speed_limit_mps": float(speed_limit),
            "samples_over_speed_limit": int(np.sum(cmd_speed > speed_limit)),
        },
        "odometry": {
            "samples": int(len(odom["t"])),
            "frequency_hz": frequency(odom["t"]),
            "speed_mps": scalar_stats(odom_speed),
        },
        "tracking": tracking_summary,
        "smooth": {
            "detected_mode": detected_mode,
            "active_intervals": [
                {"start": float(begin), "end": float(interval_end), "duration_s": float(interval_end - begin)}
                for begin, interval_end in smooth_intervals
            ],
            "super_pause_intervals": [
                {"start": float(begin), "end": float(interval_end), "duration_s": float(interval_end - begin)}
                for begin, interval_end in pause_intervals
            ],
            "command": smooth_command,
            "tracking": smooth_tracking,
            "source_handoffs": source_handoffs,
            "safety": {
                "status_transitions": safety_transitions,
                "minimum_reported_clearance_m": min(finite_clearances) if finite_clearances else None,
                "latest": data["gate_safety"][-1] if data["gate_safety"] else None,
            },
        },
        "px4ctrl": debug_summary,
        "battery": battery_summary,
        "events": {
            "goal_count": len(data["goals"]),
            "goals": data["goals"],
            "trigger_count": len(data["triggers"]),
            "takeoff_land_commands": data["takeoff_land"],
            "mode_transitions": states,
            "armed_transitions": armed,
            "landed_state_transitions": landed,
            "goal_segments": build_goal_segments(data, smooth_intervals),
            "timing": build_timing_summary(data, smooth_intervals),
        },
        "rosout": {
            "relevant_line_count": len(data["rosout"]),
            "planner_success_line_count": len(planner_successes),
            "planner_failure_line_count": len(planner_failures),
            "planner_failures": planner_failures[:100],
        },
    }


def write_tracking_csv(path, tracking, zero_time, smooth_intervals):
    if not tracking:
        return
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s", "error_x_m", "error_y_m", "error_z_m", "error_norm_m",
                "velocity_error_x_mps", "velocity_error_y_mps", "velocity_error_z_mps",
                "velocity_error_norm_mps", "yaw_error_deg", "smooth_active",
            ]
        )
        for index in range(len(tracking["t"])):
            position_error = tracking["position_error"][index]
            velocity_error = tracking["velocity_error"][index]
            writer.writerow(
                [
                    tracking["t"][index] - zero_time,
                    *position_error,
                    np.linalg.norm(position_error),
                    *velocity_error,
                    np.linalg.norm(velocity_error),
                    np.rad2deg(tracking["yaw_error"][index]),
                    int(any(begin <= tracking["t"][index] <= end
                            for begin, end in smooth_intervals)),
                ]
            )


def draw_plot(path, start, data, tracking, speed_limit, smooth_intervals):
    odom = data["odom"]
    cmd = data["cmd"]
    fig, axes = plt.subplots(4, 2, figsize=(15, 18))
    axes = axes.ravel()

    def relative(times):
        return times - start

    def add_legend(axis, *args, **kwargs):
        handles, _ = axis.get_legend_handles_labels()
        if handles:
            axis.legend(*args, **kwargs)

    for axis in axes:
        for interval_index, (begin, end) in enumerate(smooth_intervals):
            axis.axvspan(
                begin - start,
                end - start,
                color="tab:green",
                alpha=0.08,
                label="smooth active" if axis is axes[3] and interval_index == 0 else None,
            )

    if len(cmd["p"]):
        axes[0].plot(cmd["p"][:, 0], cmd["p"][:, 1], label="command", linewidth=2)
    if len(odom["p"]):
        axes[0].plot(odom["p"][:, 0], odom["p"][:, 1], label="odometry", linewidth=1)
    axes[0].set_title("XY trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    add_legend(axes[0])
    axes[0].grid(True)

    colors = ("tab:red", "tab:green", "tab:blue")
    labels = ("x", "y", "z")
    for axis_index in range(3):
        if len(cmd["t"]):
            axes[1].plot(relative(cmd["t"]), cmd["p"][:, axis_index], "--", color=colors[axis_index], label="cmd " + labels[axis_index])
        if len(odom["t"]):
            axes[1].plot(relative(odom["t"]), odom["p"][:, axis_index], color=colors[axis_index], alpha=0.7, label="odom " + labels[axis_index])
    axes[1].set_title("Position command vs odometry")
    axes[1].set_ylabel("position [m]")
    add_legend(axes[1], ncol=3, fontsize=8)
    axes[1].grid(True)

    if tracking:
        t_track = relative(tracking["t"])
        for axis_index in range(3):
            axes[2].plot(t_track, tracking["position_error"][:, axis_index], label=labels[axis_index], alpha=0.7)
        axes[2].plot(t_track, np.linalg.norm(tracking["position_error"], axis=1), color="black", label="norm")
        add_legend(axes[2], ncol=4, fontsize=8)
    else:
        axes[2].text(0.5, 0.5, "No synchronized tracking samples", ha="center", va="center", transform=axes[2].transAxes)
    axes[2].set_title("Position tracking error (odom - command)")
    axes[2].set_ylabel("error [m]")
    axes[2].grid(True)

    if len(cmd["t"]):
        axes[3].plot(relative(cmd["t"]), np.linalg.norm(cmd["v"], axis=1), label="command")
    if len(data["super_cmd"]["t"]):
        axes[3].plot(relative(data["super_cmd"]["t"]),
                     np.linalg.norm(data["super_cmd"]["v"], axis=1),
                     "--", label="SUPER source", alpha=0.7)
    if len(data["gate_cmd"]["t"]):
        axes[3].plot(relative(data["gate_cmd"]["t"]),
                     np.linalg.norm(data["gate_cmd"]["v"], axis=1),
                     ":", label="smooth source", alpha=0.9)
    if len(odom["t"]):
        axes[3].plot(relative(odom["t"]), np.linalg.norm(odom["v"], axis=1), label="odometry", alpha=0.8)
    axes[3].axhline(speed_limit, color="red", linestyle="--", label="limit {:.1f} m/s".format(speed_limit))
    axes[3].set_title("Speed")
    axes[3].set_ylabel("speed [m/s]")
    add_legend(axes[3])
    axes[3].grid(True)

    if len(cmd["t"]):
        axes[4].plot(relative(cmd["t"]), np.linalg.norm(cmd["a"], axis=1), label="acceleration")
        axes[4].plot(relative(cmd["t"]), np.linalg.norm(cmd["j"], axis=1), label="jerk")
    axes[4].set_title("Final command dynamics")
    axes[4].set_ylabel("norm")
    add_legend(axes[4])
    axes[4].grid(True)

    if tracking:
        t_track = relative(tracking["t"])
        axes[5].plot(t_track, np.linalg.norm(tracking["velocity_error"], axis=1), label="velocity error")
        yaw_axis = axes[5].twinx()
        yaw_axis.plot(t_track, np.rad2deg(tracking["yaw_error"]), color="tab:orange", alpha=0.7, label="yaw error")
        yaw_axis.set_ylabel("yaw error [deg]")
        lines, labels_left = axes[5].get_legend_handles_labels()
        lines_right, labels_right = yaw_axis.get_legend_handles_labels()
        axes[5].legend(lines + lines_right, labels_left + labels_right)
    axes[5].set_title("Velocity and yaw tracking error")
    axes[5].set_ylabel("velocity error [m/s]")
    axes[5].grid(True)

    battery = data["battery"]
    debug = data["debug"]
    if len(battery["t"]):
        axes[6].plot(relative(battery["t"]), battery["voltage"], label="battery voltage")
    if len(debug["t"]):
        voltage = debug["voltage"]
        valid = np.isfinite(voltage) & (voltage > 1.0)
        axes[6].plot(relative(debug["t"][valid]), voltage[valid], label="PX4Ctrl voltage", alpha=0.7)
    axes[6].set_title("Battery voltage")
    axes[6].set_ylabel("voltage [V]")
    add_legend(axes[6])
    axes[6].grid(True)

    if len(debug["t"]):
        axes[7].plot(relative(debug["t"]), np.abs(debug["att_err"]), label="attitude error [deg]")
        rate_error = np.linalg.norm(debug["fb_rate"] - debug["ideal_rate"], axis=1)
        axes[7].plot(relative(debug["t"]), rate_error, label="body-rate error [rad/s]")
    target = data["attitude_target"]
    if len(target["t"]):
        axes[7].plot(relative(target["t"]), target["thrust"], label="normalized thrust", alpha=0.7)
    axes[7].set_title("PX4Ctrl execution")
    add_legend(axes[7])
    axes[7].grid(True)

    for axis in axes[1:]:
        axis.set_xlabel("bag time [s]")
    fig.tight_layout()
    fig.savefig(str(path), dpi=160, bbox_inches="tight")
    plt.close(fig)


def write_text_summary(path, summary):
    command = summary["command"]
    odometry = summary["odometry"]
    tracking = summary["tracking"]
    px4ctrl = summary["px4ctrl"]
    battery = summary["battery"]
    rosout = summary["rosout"]
    smooth = summary["smooth"]

    lines = [
        "SUPER 实机飞行 bag 分析",
        "=" * 64,
        "bag: {}".format(", ".join(summary["bags"])),
        "记录时长: {}".format(fmt(summary["recording"]["duration_s"], suffix=" s")),
        "里程计: {}，{} 条，约 {}".format(
            summary["selected_topics"]["odom"], odometry["samples"], fmt(odometry["frequency_hz"], suffix=" Hz")
        ),
        "规划指令: {}，{} 条，约 {}".format(
            summary["selected_topics"]["cmd"], command["samples"], fmt(command["frequency_hz"], suffix=" Hz")
        ),
        "坐标系: odom={}，command={}".format(summary["frames"]["odom"], summary["frames"]["command"]),
        "",
        "规划指令质量",
        "- 有效指令跨度: {}".format(fmt(command["active_span_s"], suffix=" s")),
        "- 最大/平均指令速度: {} / {}".format(
            fmt(command["speed_mps"]["max"] if command["speed_mps"] else None, suffix=" m/s"),
            fmt(command["speed_mps"]["mean"] if command["speed_mps"] else None, suffix=" m/s"),
        ),
        "- 最大指令加速度: {}".format(
            fmt(command["acceleration_mps2"]["max"] if command["acceleration_mps2"] else None, suffix=" m/s^2")
        ),
        "- 最大指令 jerk: {}".format(
            fmt(command["jerk_mps3"]["max"] if command["jerk_mps3"] else None, suffix=" m/s^3")
        ),
        "- backup 占比: {}".format(
            fmt(100.0 * command["backup_ratio"] if command["backup_ratio"] is not None else None, suffix=" %")
        ),
        "- 超过 {:.2f} m/s 的指令样本: {}".format(command["speed_limit_mps"], command["samples_over_speed_limit"]),
        "- 大于 0.2 s 的指令中断: {}".format(command["gap_count_over_0_2s"]),
        "",
        "实际飞行与跟踪",
        "- 全 bag 最大里程计速度: {}".format(
            fmt(odometry["speed_mps"]["max"] if odometry["speed_mps"] else None, suffix=" m/s")
        ),
    ]
    if tracking:
        lines.extend(
            [
                "- 最终规划指令期间最大实际速度: {}，超过 {:.2f} m/s 的样本: {}".format(
                    fmt(tracking["actual_speed_mps"]["max"], suffix=" m/s"),
                    command["speed_limit_mps"],
                    tracking["actual_samples_over_speed_limit"],
                ),
                "- 位置误差 RMSE / P95 / 最大值: {} / {} / {}".format(
                    fmt(tracking["position_error_m"]["rmse"], suffix=" m"),
                    fmt(tracking["position_error_m"]["p95"], suffix=" m"),
                    fmt(tracking["position_error_m"]["max"], suffix=" m"),
                ),
                "- XYZ 位置误差 RMSE: {} m".format(
                    ", ".join("{:.3f}".format(value) for value in tracking["position_axis_rmse_m"])
                ),
                "- 速度误差 RMSE / 最大值: {} / {}".format(
                    fmt(tracking["velocity_error_mps"]["rmse"], suffix=" m/s"),
                    fmt(tracking["velocity_error_mps"]["max"], suffix=" m/s"),
                ),
                "- 航向误差 RMSE / 最大值: {} / {}".format(
                    fmt(tracking["yaw_error_deg"]["rmse"], suffix=" deg"),
                    fmt(tracking["yaw_error_deg"]["max"], suffix=" deg"),
                ),
            ]
        )
    else:
        lines.append("- 没有得到同步的指令/里程计样本，无法计算跟踪误差。")

    lines.extend(
        [
            "",
            "模式与 smooth/D 区专项",
            "- 自动识别模式: {}".format(smooth["detected_mode"]),
            "- smooth 接管次数/累计时长（hybrid 为 C 出口至降落区）: {} / {}".format(
                len(smooth["active_intervals"]),
                fmt(sum(item["duration_s"] for item in smooth["active_intervals"]), suffix=" s"),
            ),
            "- SUPER pause 次数/累计时长: {} / {}".format(
                len(smooth["super_pause_intervals"]),
                fmt(sum(item["duration_s"] for item in smooth["super_pause_intervals"]), suffix=" s"),
            ),
        ]
    )
    if smooth["command"]:
        lines.extend(
            [
                "- smooth 区间最大指令速度/加速度/jerk: {} / {} / {}".format(
                    fmt(smooth["command"]["speed_mps"]["max"], suffix=" m/s"),
                    fmt(smooth["command"]["acceleration_mps2"]["max"], suffix=" m/s^2"),
                    fmt(smooth["command"]["jerk_mps3"]["max"], suffix=" m/s^3"),
                ),
                "- smooth 区间超过 {:.2f} m/s 的指令样本: {}".format(
                    command["speed_limit_mps"], smooth["command"]["samples_over_speed_limit"]
                ),
            ]
        )
    if smooth["tracking"]:
        smooth_tracking = smooth["tracking"]
        lines.extend(
            [
                "- smooth 区间位置误差 RMSE / P95 / 最大值: {} / {} / {}".format(
                    fmt(smooth_tracking["position_error_m"]["rmse"], suffix=" m"),
                    fmt(smooth_tracking["position_error_m"]["p95"], suffix=" m"),
                    fmt(smooth_tracking["position_error_m"]["max"], suffix=" m"),
                ),
                "- smooth 区间最大实际速度: {}，超限样本: {}".format(
                    fmt(smooth_tracking["actual_speed_mps"]["max"], suffix=" m/s"),
                    smooth_tracking["actual_samples_over_speed_limit"],
                ),
            ]
        )
    if smooth["source_handoffs"]:
        handoff = smooth["source_handoffs"]
        lines.append(
            "- SUPER→smooth 最大位置/速度/加速度指令跳变: {} / {} / {}".format(
                fmt(handoff["max_position_jump_m"], suffix=" m"),
                fmt(handoff["max_velocity_jump_mps"], suffix=" m/s"),
                fmt(handoff["max_acceleration_jump_mps2"], suffix=" m/s^2"),
            )
        )
    safety = smooth["safety"]
    lines.append(
        "- D 区包络最小报告净空/最新状态: {} / {}".format(
            fmt(safety["minimum_reported_clearance_m"], suffix=" m"),
            safety["latest"]["status"] if safety["latest"] else "N/A",
        )
    )

    lines.extend(["", "PX4Ctrl 与飞控"])
    if px4ctrl:
        lines.extend(
            [
                "- 姿态执行误差 RMSE / 最大值: {} / {}".format(
                    fmt(px4ctrl["attitude_error_deg"]["rmse"] if px4ctrl["attitude_error_deg"] else None, suffix=" deg"),
                    fmt(px4ctrl["attitude_error_deg"]["max"] if px4ctrl["attitude_error_deg"] else None, suffix=" deg"),
                ),
                "- 角速度误差 RMSE / 最大值: {} / {}".format(
                    fmt(px4ctrl["body_rate_error_radps"]["rmse"] if px4ctrl["body_rate_error_radps"] else None, suffix=" rad/s"),
                    fmt(px4ctrl["body_rate_error_radps"]["max"] if px4ctrl["body_rate_error_radps"] else None, suffix=" rad/s"),
                ),
            ]
        )
    else:
        lines.append("- bag 中没有 /debugPx4ctrl，无法分析控制器内部量。")
    if battery:
        lines.append(
            "- 电池电压 起始/最低/结束: {} / {} / {}".format(
                fmt(battery["start_v"], suffix=" V"), fmt(battery["min_v"], suffix=" V"), fmt(battery["end_v"], suffix=" V")
            )
        )
    else:
        lines.append("- 没有有效电池电压。")

    events = summary["events"]
    timing = events["timing"]
    lines.extend(
        [
            "",
            "任务与诊断事件",
            "- 目标点消息: {}，起飞后任务触发: {}，起降命令: {}".format(
                events["goal_count"], events["trigger_count"], len(events["takeoff_land_commands"])
            ),
            "- MAVROS 模式切换: {}".format(
                " -> ".join(item["mode"] or "<empty>" for item in events["mode_transitions"]) or "N/A"
            ),
            "- /rosout 中匹配到的规划成功行: {}，规划失败行: {}".format(
                rosout["planner_success_line_count"], rosout["planner_failure_line_count"]
            ),
            "- TAKEOFF→任务触发 / 触发→首条指令: {} / {}".format(
                fmt(timing["takeoff_command_to_trigger_s"], suffix=" s"),
                fmt(timing["trigger_to_first_command_s"], suffix=" s"),
            ),
            "- LAND→ON_GROUND / LAND→disarm: {} / {}".format(
                fmt(timing["land_request_to_on_ground_s"], suffix=" s"),
                fmt(timing["land_request_to_disarm_s"], suffix=" s"),
            ),
        ]
    )
    if events["goal_segments"]:
        lines.append("- 各目标点从发布到下一事件的时间：")
        for segment in events["goal_segments"]:
            lines.append(
                "  #{:02d} [{:.2f}, {:.2f}, {:.2f}] {}".format(
                    segment["index"], *segment["position"],
                    fmt(segment["duration_until_next_event_s"], suffix=" s"),
                )
            )
    if rosout["planner_failures"]:
        lines.append("- 规划失败摘要（最多列出前 10 条）：")
        for item in rosout["planner_failures"][:10]:
            lines.append("  [{:.3f}] {}".format(item["t"], item["message"]))

    lines.extend(
        [
            "",
            "说明",
            "- 同步优先使用消息 header.stamp，无效时退回 rosbag 接收时间；只有距最近 PositionCommand 不超过设定阈值的里程计样本参与误差计算。",
            "- 位置误差假定里程计与 PositionCommand 位于同一 world 坐标系。",
            "- 本报告能分析闭环跟踪和控制状态，但不能替代现场安全检查。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    if args.sync_max_gap <= 0:
        raise SystemExit("错误：--sync-max-gap 必须大于 0。")
    if args.speed_limit <= 0:
        raise SystemExit("错误：--speed-limit 必须大于 0。")

    try:
        bag_paths = resolve_bags(args.bags, WORKSPACE)
        available, topic_counts, start, end = inspect_bags(bag_paths)
        odom_topic = select_topic(args.odom_topic, ODOM_CANDIDATES, available, "里程计")
        cmd_topic = select_topic(args.cmd_topic, CMD_CANDIDATES, available, "规划指令")
    except (FileNotFoundError, RuntimeError, rosbag.ROSBagException) as exc:
        print("错误：{}".format(exc), file=sys.stderr)
        return 1

    topics = {
        "odom": odom_topic,
        "cmd": cmd_topic,
        "debug": DEBUG_TOPIC if DEBUG_TOPIC in available else None,
        "battery": BATTERY_TOPIC if BATTERY_TOPIC in available else None,
        "imu": IMU_TOPIC if IMU_TOPIC in available else None,
        "attitude_target": ATTITUDE_TARGET_TOPIC if ATTITUDE_TARGET_TOPIC in available else None,
        "state": STATE_TOPIC if STATE_TOPIC in available else None,
        "extended_state": EXTENDED_STATE_TOPIC if EXTENDED_STATE_TOPIC in available else None,
        "goal": GOAL_TOPIC if GOAL_TOPIC in available else None,
        "trigger": TRIGGER_TOPIC if TRIGGER_TOPIC in available else None,
        "takeoff_land": TAKEOFF_LAND_TOPIC if TAKEOFF_LAND_TOPIC in available else None,
        "super_cmd": SUPER_CMD_TOPIC if SUPER_CMD_TOPIC in available else None,
        "gate_cmd": GATE_CMD_TOPIC if GATE_CMD_TOPIC in available else None,
        "gate_active": GATE_ACTIVE_TOPIC if GATE_ACTIVE_TOPIC in available else None,
        "super_pause": SUPER_PAUSE_TOPIC if SUPER_PAUSE_TOPIC in available else None,
        "gate_markers": GATE_MARKERS_TOPIC if GATE_MARKERS_TOPIC in available else None,
        "rosout": ROSOUT_TOPIC if ROSOUT_TOPIC in available else None,
    }

    try:
        data = convert_arrays(read_data(bag_paths, topics))
    except RuntimeError as exc:
        print("错误：{}".format(exc), file=sys.stderr)
        return 1

    tracking = compute_tracking(data, args.sync_max_gap)
    summary = build_summary(
        bag_paths, available, topic_counts, start, end, topics, data, tracking, args.speed_limit
    )
    smooth_intervals = [
        (item["start"], item["end"]) for item in summary["smooth"]["active_intervals"]
    ]

    if args.output:
        output_dir = Path(args.output).expanduser().resolve()
    else:
        output_dir = bag_paths[0].parent / (bag_paths[0].stem + "_analysis")
    output_dir.mkdir(parents=True, exist_ok=True)

    json_path = output_dir / "summary.json"
    text_path = output_dir / "summary.txt"
    csv_path = output_dir / "tracking_error.csv"
    plot_path = output_dir / "flight_analysis.png"

    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_text_summary(text_path, summary)
    write_tracking_csv(csv_path, tracking, start, smooth_intervals)
    draw_plot(plot_path, start, data, tracking, args.speed_limit, smooth_intervals)

    print(text_path.read_text(encoding="utf-8"))
    print("分析结果已保存到：{}".format(output_dir))
    print("- {}".format(text_path.name))
    print("- {}".format(json_path.name))
    if tracking:
        print("- {}".format(csv_path.name))
    print("- {}".format(plot_path.name))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
