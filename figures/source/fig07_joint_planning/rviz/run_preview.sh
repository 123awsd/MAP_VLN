#!/usr/bin/env bash
# Isolated RViz playback of this figure's exact saved trajectory. No control topics.
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/../../.." && pwd)"
preview_job="$(mktemp -d /tmp/paper-jtvp-rviz.XXXXXX)"
preview_uid="$(id -u)"
preview_gid="$(id -g)"
preview_auth="${XAUTHORITY:-/run/user/$preview_uid/gdm/Xauthority}"
[[ -f "$preview_auth" ]] || { echo "X11 authorization file not found: $preview_auth" >&2; exit 1; }
preview_docker=(docker)
if ! docker info >/dev/null 2>&1; then preview_docker=(sudo docker); fi
"${preview_docker[@]}" run --rm --network none --user "$preview_uid:$preview_gid" \
  --cap-drop ALL --security-opt no-new-privileges \
  --tmpfs "/.rviz:uid=$preview_uid,gid=$preview_gid" --entrypoint /bin/bash \
  -v "$project_root:/workspace/project:ro" -v "$preview_job:/job" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$preview_auth:/xauth:ro" \
  -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/xauth \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 \
  pre-map-vln/real-minco:local -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/job/ros
roscore > /job/roscore.log 2>&1 & master=$!
trap "kill $master 2>/dev/null || true" EXIT
for i in $(seq 1 50); do rosparam list >/dev/null 2>&1 && break; sleep .1; done
g++ -std=c++14 -fPIC /workspace/project/画图/联合任务视角规划/rviz/capture.cpp -o /job/capture $(pkg-config --cflags --libs rviz Qt5Widgets OGRE)
python3 /workspace/project/画图/联合任务视角规划/rviz/preview.py --publish-only > /job/publisher.log 2>&1 & publisher=$!
trap "kill $publisher $master 2>/dev/null || true" EXIT
/job/capture /workspace/project/画图/联合任务视角规划/rviz/scene.rviz /job/rviz_preview.png
'
echo "RViz screenshot: $preview_job/rviz_preview.png"
