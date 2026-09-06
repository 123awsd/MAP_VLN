#!/usr/bin/env bash
# Pure offline replay of recorded mapping outputs and RGB; starts no sensor or flight node.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
stage1_dir="$(cd "$script_dir/.." && pwd)"
root="$(cd "$stage1_dir/../.." && pwd)"
dataset="${1:-Drone_room_02}"
rate="${2:-0.25}"
loop="${3:-false}"
bag="$stage1_dir/data/$dataset/raw/sensors.bag"
rviz_config="$stage1_dir/rviz/drone_room_bag_replay.rviz"
image="pre-map-vln/falcon-noetic:local"

[[ -f "$bag" ]] || { echo "Missing bag: $bag" >&2; exit 2; }
[[ "$loop" == "true" || "$loop" == "false" ]] || {
  echo "Usage: $0 [DATASET] [RATE] [true|false]" >&2; exit 2;
}
[[ "$rate" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Playback rate must be positive" >&2; exit 2; }

"$root/scripts/prepare_rviz_xauth.sh"
docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then docker_cmd=(sudo -n docker); fi
echo "Offline replay only: no MAVROS, PX4Ctrl, sensor driver, setpoint, or flight node."
echo "Dataset=$dataset rate=${rate}x; RGB is enabled and raw depth can be toggled in Displays."
exec "${docker_cmd[@]}" run --rm --init --net host \
  -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/root/.Xauthority \
  -e REPLAY_DATASET="$dataset" -e REPLAY_RATE="$rate" -e REPLAY_LOOP="$loop" \
  -v "$root:/workspace/project:ro" \
  -v "$root/runtime/rviz.Xauthority:/root/.Xauthority:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  "$image" bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    roscore >/tmp/drone_room_replay_roscore.log 2>&1 & master=$!
    rviz_pid=""
    trap "kill $rviz_pid $master 2>/dev/null || true" EXIT INT TERM
    sleep 2
    rosparam set use_sim_time true
    rviz -d /workspace/project/real_fly/stage1_exploration/rviz/drone_room_bag_replay.rviz \
      >/tmp/drone_room_replay_rviz.log 2>&1 & rviz_pid=$!
    sleep 3
    loop_args=()
    [[ "$REPLAY_LOOP" == "true" ]] && loop_args=(--loop)
    rosbag play --clock --rate "$REPLAY_RATE" "${loop_args[@]}" \
      "/workspace/project/real_fly/stage1_exploration/data/$REPLAY_DATASET/raw/sensors.bag" \
      --topics /cloud_registered /Odometry /tf /tf_static \
               /camera/color/image_raw /camera/depth/image_rect_raw
  '
