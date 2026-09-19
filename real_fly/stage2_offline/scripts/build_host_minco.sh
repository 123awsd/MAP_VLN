#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
image=pre-map-vln/real-minco:local
if ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build -t "$image" -f "$root/real_fly/stage2_offline/Dockerfile.minco" "$root/real_fly/stage2_offline"
fi
mkdir -p "$root/real_fly/stage2_offline/runtime/minco_ws"
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -v "$root:/workspace/project" "$image" bash -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
ws=/workspace/project/real_fly/stage2_offline/runtime/minco_ws
mkdir -p "$ws/src"
for pkg in quadrotor_msgs rog_map super_planner mission_planner; do
  ln -sfn "/workspace/project/real_fly/vendor/dls_ws_src/$pkg" "$ws/src/$pkg"
done
cd "$ws"
catkin_make -DCATKIN_WHITELIST_PACKAGES="quadrotor_msgs;rog_map;super_planner;mission_planner" -j2 full_smooth_mission
'
