#!/usr/bin/env python3
"""Read-only single-UAV preflight and runtime terminal monitor."""

import argparse
import collections
import math
import os
import threading
import time

import rosgraph
import rospy
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, Px4ctrlDebug
from sensor_msgs.msg import BatteryState, Imu
from mavros_msgs.msg import AttitudeTarget


CTRL_NAMES = {
    1: "MANUAL_CTRL",
    2: "AUTO_HOVER",
    3: "CMD_CTRL",
    4: "AUTO_TAKEOFF",
    5: "AUTO_LAND",
}
WINDOW_SECONDS = 5.0


def finite(*values):
    return all(math.isfinite(value) for value in values)


def fmt_age(value):
    return "--" if math.isinf(value) else f"{value * 1000:5.0f} ms"


class Stream:
    def __init__(self):
        self.arrivals = collections.deque()
        self.last = None
        self.message = None

    def feed(self, message):
        now = time.monotonic()
        self.arrivals.append(now)
        self.last = now
        self.message = message
        self.trim(now)

    def trim(self, now):
        cutoff = now - WINDOW_SECONDS
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()

    def rate(self, now):
        self.trim(now)
        if len(self.arrivals) < 2:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0

    def age(self, now):
        return math.inf if self.last is None else now - self.last


class Monitor:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.streams = {name: Stream() for name in (
            "state", "battery", "rc", "imu", "odom", "ctrl", "cmd", "attitude"
        )}
        self.odom_history = collections.deque()
        self.odom_delay_ms = math.inf
        self.publishers = {}
        self.last_graph_query = 0.0

        rospy.Subscriber("/mavros/state", State, self.callback("state"), queue_size=20)
        rospy.Subscriber("/mavros/battery", BatteryState, self.callback("battery"), queue_size=20)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.callback("rc"), queue_size=30)
        rospy.Subscriber("/mavros/imu/data", Imu, self.callback("imu"), queue_size=400)
        rospy.Subscriber("/ekf_quat/ekf_odom", Odometry, self.odom_callback, queue_size=400)
        rospy.Subscriber("/debugPx4ctrl", Px4ctrlDebug, self.callback("ctrl"), queue_size=200)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.callback("cmd"), queue_size=200)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget,
                         self.callback("attitude"), queue_size=200)

    def callback(self, name):
        def receive(message):
            with self.lock:
                self.streams[name].feed(message)
        return receive

    def odom_callback(self, message):
        now = time.monotonic()
        stamp = message.header.stamp.to_sec()
        delay = max(0.0, (rospy.Time.now().to_sec() - stamp) * 1000.0) if stamp else math.inf
        p = message.pose.pose.position
        with self.lock:
            self.streams["odom"].feed(message)
            self.odom_delay_ms = delay
            self.odom_history.append((now, (p.x, p.y, p.z)))
            cutoff = now - WINDOW_SECONDS
            while self.odom_history and self.odom_history[0][0] < cutoff:
                self.odom_history.popleft()

    def update_graph(self, now):
        if now - self.last_graph_query < 1.0:
            return
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            self.publishers = {topic: nodes for topic, nodes in publishers}
        except Exception:
            self.publishers = {}
        self.last_graph_query = now

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            rates = {name: stream.rate(now) for name, stream in self.streams.items()}
            ages = {name: stream.age(now) for name, stream in self.streams.items()}
            messages = {name: stream.message for name, stream in self.streams.items()}
            odom_history = list(self.odom_history)
            odom_delay_ms = self.odom_delay_ms
        self.update_graph(now)
        return now, rates, ages, messages, odom_history, odom_delay_ms

    @staticmethod
    def rc_switch(pwm):
        if pwm is None:
            return "--"
        if pwm > 1750:
            return "ON"
        if pwm < 1250:
            return "OFF"
        return "MID(!)"

    def display(self):
        _, rates, ages, msg, history, odom_delay = self.snapshot()
        failures = []
        warnings = []
        min_voltage = (self.args.min_voltage if self.args.min_voltage is not None else
                       self.args.battery_cells * self.args.min_cell_voltage)

        state = msg["state"]
        connected = bool(state and ages["state"] < 1.0 and state.connected)
        armed = bool(state and state.armed)
        px4_mode = state.mode if state and ages["state"] < 1.0 else "--"
        if not connected:
            failures.append("MAVROS未收到新鲜的飞控连接状态")

        battery = msg["battery"]
        voltage = battery.voltage if battery and finite(battery.voltage) else math.nan
        current = battery.current if battery and finite(battery.current) else math.nan
        percentage = battery.percentage * 100.0 if (
            battery and finite(battery.percentage) and battery.percentage >= 0
        ) else math.nan
        if ages["battery"] >= 2.0:
            failures.append("电池数据缺失或过期")
        elif not math.isfinite(voltage) or voltage < min_voltage:
            failures.append(
                f"{self.args.battery_cells}S电池电压低于{min_voltage:.1f} V或读数无效")

        rc = msg["rc"]
        channels = list(rc.channels) if rc else []
        ch5 = channels[4] if len(channels) > 4 else None
        ch6 = channels[5] if len(channels) > 5 else None
        if ages["rc"] >= 0.5 or rates["rc"] < self.args.min_rc_hz:
            failures.append("遥控器数据不新鲜或频率过低")
        if self.rc_switch(ch5) == "MID(!)" or self.rc_switch(ch6) == "MID(!)":
            failures.append("5/6通道处于中间区，拨杆状态不明确")

        imu_ok = ages["imu"] < 0.5 and rates["imu"] >= self.args.min_imu_hz
        if not imu_ok:
            failures.append("飞控IMU数据不新鲜或频率过低")

        odom = msg["odom"]
        speed = math.inf
        position = (math.nan, math.nan, math.nan)
        frame = "--"
        if odom:
            p = odom.pose.pose.position
            v = odom.twist.twist.linear
            position = (p.x, p.y, p.z)
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            frame = odom.header.frame_id
        jumps = [math.dist(history[index - 1][1], history[index][1])
                 for index in range(1, len(history))]
        max_jump = max(jumps, default=0.0)
        odom_ok = ages["odom"] < 0.5 and rates["odom"] >= self.args.min_odom_hz
        if not odom_ok:
            failures.append("EKF里程计不新鲜或频率过低")
        if frame != "world":
            failures.append(f"里程计坐标系为{frame!r}，不是'world'")
        if not finite(*position, speed):
            failures.append("里程计包含非有限值")
        if max_jump > self.args.max_jump:
            failures.append(f"5秒内位置单帧跳变{max_jump:.2f} m")
        if math.isfinite(odom_delay) and odom_delay > self.args.max_odom_delay_ms:
            failures.append(f"里程计延迟{odom_delay:.0f} ms过高")

        ctrl = msg["ctrl"]
        ctrl_state = int(ctrl.state) if ctrl and ages["ctrl"] < 0.5 else 0
        ctrl_name = CTRL_NAMES.get(ctrl_state, "--")
        if not ctrl_state or rates["ctrl"] < self.args.min_ctrl_hz:
            failures.append("PX4Ctrl状态不新鲜或频率过低")
        if ctrl_state in (2, 3, 4, 5) and px4_mode != "OFFBOARD":
            failures.append(f"PX4Ctrl为{ctrl_name}但PX4不是OFFBOARD")
        if ctrl_state == 1 and px4_mode == "OFFBOARD":
            warnings.append("PX4为OFFBOARD但PX4Ctrl仍在MANUAL_CTRL")

        command_active = ages["cmd"] < 0.2
        attitude_active = ages["attitude"] < 0.2
        command_publishers = self.publishers.get("/planning/pos_cmd", [])
        if not armed and command_active:
            failures.append("未解锁时/planning/pos_cmd仍在发送，禁止拨5通道")
        if ctrl_state in (2, 3, 4, 5) and not attitude_active:
            failures.append("PX4Ctrl接管状态下姿态指令没有持续发送")

        if not armed and math.isfinite(speed) and speed > self.args.max_stationary_speed:
            failures.append(f"地面静止速度{speed:.2f} m/s过大")

        if armed:
            phase = "飞行中监控"
            ready = not failures
        else:
            phase = "起飞前就绪" if not failures else "禁止起飞"
            ready = not failures

        clear = "\033[H\033[2J" if os.isatty(1) else ""
        print(clear, end="")
        print("================ 单机飞行状态（只读） ================")
        print(f"结论: {'PASS' if ready else 'FAIL'} / {phase}    刷新: {time.strftime('%H:%M:%S')}")
        print("------------------------------------------------------")
        print(f"飞控  connected={connected!s:<5} armed={armed!s:<5} PX4={px4_mode:<12} age={fmt_age(ages['state'])}")
        print(f"电池  {voltage:5.2f} V  {current:6.2f} A  {percentage:5.1f}%  "
              f"{self.args.battery_cells}S(min={min_voltage:.1f}V)  age={fmt_age(ages['battery'])}")
        print(f"遥控  {rates['rc']:6.1f} Hz  CH5={self.rc_switch(ch5):<6}({ch5 or 0:4})  CH6={self.rc_switch(ch6):<6}({ch6 or 0:4})")
        print("------------------------------------------------------")
        print(f"定位  frame={frame:<8} xyz=({position[0]:7.2f},{position[1]:7.2f},{position[2]:7.2f})")
        print(f"      speed={speed:6.3f} m/s  {rates['odom']:6.1f} Hz  age={fmt_age(ages['odom'])}")
        print(f"      delay={odom_delay:6.1f} ms  max_step_5s={max_jump:6.3f} m")
        print(f"IMU   {rates['imu']:6.1f} Hz  age={fmt_age(ages['imu'])}")
        print("------------------------------------------------------")
        print(f"控制  PX4Ctrl={ctrl_name:<13} {rates['ctrl']:6.1f} Hz  age={fmt_age(ages['ctrl'])}")
        print(f"      planner_cmd={'ACTIVE' if command_active else 'idle':<6} {rates['cmd']:6.1f} Hz  publishers={','.join(command_publishers) or '--'}")
        print(f"      attitude={'ACTIVE' if attitude_active else 'idle':<6} {rates['attitude']:6.1f} Hz  age={fmt_age(ages['attitude'])}")
        print("------------------------------------------------------")
        if failures:
            for reason in dict.fromkeys(failures):
                print(f"[FAIL] {reason}")
        else:
            print("[ OK ] 所有当前阶段硬门槛通过")
        for reason in dict.fromkeys(warnings):
            print(f"[WARN] {reason}")
        print("只读监视器：不会发布话题、调用服务、切模式、解锁或起飞。Ctrl-C退出。")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--battery-cells", type=int, default=4,
                        help="configured pack cell count; do not infer this from voltage")
    parser.add_argument("--min-cell-voltage", type=float, default=3.5)
    parser.add_argument("--min-voltage", type=float, default=None,
                        help="optional explicit pack-voltage threshold override")
    parser.add_argument("--min-odom-hz", type=float, default=100.0)
    parser.add_argument("--min-imu-hz", type=float, default=100.0)
    parser.add_argument("--min-rc-hz", type=float, default=5.0)
    parser.add_argument("--min-ctrl-hz", type=float, default=50.0)
    parser.add_argument("--max-jump", type=float, default=0.30)
    parser.add_argument("--max-stationary-speed", type=float, default=0.10)
    parser.add_argument("--max-odom-delay-ms", type=float, default=150.0)
    args = parser.parse_args()
    if args.battery_cells < 1 or not 2.5 <= args.min_cell_voltage <= 4.2:
        parser.error("invalid battery cell count or per-cell voltage threshold")
    if args.min_voltage is not None and args.min_voltage <= 0:
        parser.error("--min-voltage must be positive")
    rospy.init_node("pre_map_vln_single_uav_monitor", anonymous=False)
    monitor = Monitor(args)
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        monitor.display()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
