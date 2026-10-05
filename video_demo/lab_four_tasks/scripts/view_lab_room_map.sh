#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
"$root_dir/PRE_MAP_VLN/scripts/prepare_rviz_xauth.sh" >/dev/null
exec docker run --rm --init --net host \
 --device nvidia.com/gpu=all -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
 -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/root/.Xauthority \
 -e DISABLE_ROS1_EOL_WARNINGS=1 -e QT_X11_NO_MITSHM=1 \
 -e ROS_MASTER_URI=http://127.0.0.1:11329 \
 -v "$root_dir:/workspace/project:ro" \
 -v "$root_dir/PRE_MAP_VLN/runtime/rviz.Xauthority:/root/.Xauthority:ro" \
 -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
 pre-map-vln/falcon-noetic:local bash -lc '
 set -e
 source /opt/ros/noetic/setup.bash
 roscore -p 11329 >/tmp/room_master.log 2>&1 & master=$!
 publisher=""
 trap "kill $master $publisher 2>/dev/null || true" EXIT
 sleep 2
 python3 /workspace/project/PRE_MAP_VLN/real_fly/stage2_offline/rviz/publish_lab_rooms.py \
 /workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/approved_scene_graph.json \
 /workspace/project/真机实验素材/成功场次/实验室4任务加主动搜索/04_最终轨迹/final_minco_preview.json & publisher=$!
 rviz -d /workspace/project/PRE_MAP_VLN/real_fly/stage2_offline/rviz/lab_room_map.rviz
 '
