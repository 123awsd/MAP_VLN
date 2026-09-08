#!/usr/bin/env bash
set -euo pipefail

# Read-only FCU telemetry monitor. It may start roscore and MAVROS when absent,
# but never starts PX4Ctrl and never publishes commands or setpoints.

ROS_MASTER_PORT=11311
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$STAGE1_ROOT/runtime/battery_monitor/$RUN_STAMP"
mkdir -p "$LOG_DIR"

unset _CATKIN_SETUP_DIR || true
source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1

owned_pids=()
cleanup() {
  local pid idx
  trap - EXIT INT TERM
  set +e
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    pid="${owned_pids[$idx]}"
    kill -INT "$pid" 2>/dev/null || true
  done
  for ((idx=${#owned_pids[@]}-1; idx>=0; idx--)); do
    wait "${owned_pids[$idx]}" 2>/dev/null || true
  done
  printf '\n电量监控已退出。\n'
}
on_signal() {
  cleanup
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

start_owned() {
  local log_file="$1"
  shift
  "$@" >"$LOG_DIR/$log_file" 2>&1 &
  owned_pids+=("$!")
}

node_exists() {
  rosnode list 2>/dev/null | grep -Fxq "$1"
}

if ! rosnode list >/dev/null 2>&1; then
  echo "未发现 ROS master，正在启动……"
  start_owned roscore.log roscore -p "$ROS_MASTER_PORT"
  for _ in {1..15}; do
    rosnode list >/dev/null 2>&1 && break
    sleep 1
  done
  rosnode list >/dev/null 2>&1 || {
    echo "ROS master 启动失败，日志：$LOG_DIR/roscore.log" >&2
    exit 1
  }
fi

if node_exists /mavros; then
  echo "正在复用已有 MAVROS。"
else
  echo "未发现 MAVROS，正在启动只读飞控遥测（/dev/ttyTHS0:921600）……"
  start_owned mavros.log roslaunch mavros px4.launch
fi

echo "不会启动 PX4Ctrl，不会解锁，也不会发送运动指令。"
echo "按 Ctrl+C 退出。"
sleep 2

last_connected="--"
last_armed="--"
last_mode="--"
last_voltage="--"
last_current="--"
last_percentage="--"
last_battery_epoch=0
last_battery_time="尚未收到"

while true; do
  state_msg="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
  battery_msg="$(timeout 2 rostopic echo -n 1 /mavros/battery 2>/dev/null || true)"

  if [[ -n "$state_msg" ]]; then
    connected="$(awk '$1 == "connected:" {print $2; exit}' <<<"$state_msg")"
    armed="$(awk '$1 == "armed:" {print $2; exit}' <<<"$state_msg")"
    mode="$(awk '$1 == "mode:" {print $2; exit}' <<<"$state_msg")"
    [[ -n "$connected" ]] && last_connected="$connected"
    [[ -n "$armed" ]] && last_armed="$armed"
    [[ -n "$mode" ]] && last_mode="$mode"
  fi

  if [[ -n "$battery_msg" ]]; then
    voltage="$(awk '$1 == "voltage:" {print $2; exit}' <<<"$battery_msg")"
    current="$(awk '$1 == "current:" {print $2; exit}' <<<"$battery_msg")"
    percentage="$(awk '$1 == "percentage:" {print $2; exit}' <<<"$battery_msg")"

    [[ "$voltage" =~ ^-?[0-9]+([.][0-9]+)?$ ]] && last_voltage="$voltage"
    [[ "$current" =~ ^-?[0-9]+([.][0-9]+)?$ ]] && last_current="$current"
    if [[ "$percentage" =~ ^-?[0-9]+([.][0-9]+)?$ ]] && awk -v p="$percentage" 'BEGIN {exit !(p >= 0)}'; then
      last_percentage="$(awk -v p="$percentage" 'BEGIN {printf "%.0f%%", p * 100}')"
    fi
    last_battery_epoch="$(date +%s)"
    last_battery_time="$(date '+%Y-%m-%d %H:%M:%S')"
  fi

  now_epoch="$(date +%s)"
  if (( last_battery_epoch > 0 )); then
    battery_age="$((now_epoch - last_battery_epoch)) 秒前"
  else
    battery_age="--"
  fi

  printf '\033[H\033[2J'
  echo "========== 无人机状态 =========="
  printf '飞控连接 : %s\n' "$last_connected"
  printf '解锁状态 : %s\n' "$last_armed"
  printf '飞行模式 : %s\n' "$last_mode"
  echo "--------------------------------"
  printf '电池电压 : %s V\n' "$last_voltage"
  printf '当前电流 : %s A\n' "$last_current"
  printf '剩余电量 : %s\n' "$last_percentage"
  printf '最后更新 : %s（%s）\n' "$last_battery_time" "$battery_age"
  echo "================================"

  if [[ -z "$state_msg" ]]; then
    echo "等待飞控数据：请确认飞控已上电并连接串口。"
  elif [[ -z "$battery_msg" ]]; then
    echo "本轮未收到新电池数据，以上保留最近一次有效读数。"
  fi
  sleep 1
done
