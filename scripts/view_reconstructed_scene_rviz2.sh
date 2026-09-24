#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_dir="${1:-$root_dir/../真机/reconstruction_readable}"
scene_dir="$(realpath "$scene_dir")"
config_path="$root_dir/config/reconstructed_scene_ros2.rviz"

for required in \
  map_rgb_colored_5cm.ply \
  map_rgb_roofless_clean_5cm.ply \
  map_building_clean_5cm.ply \
  map_ground_5cm.ply \
  map_walls_5cm.ply \
  map_ceiling_5cm.ply; do
  if [[ ! -f "$scene_dir/$required" ]]; then
    echo "missing reconstructed cloud: $scene_dir/$required" >&2
    exit 1
  fi
done
if [[ ! -f "$scene_dir/occupancy_2d/occupancy_grid.npz" ]]; then
  echo "missing occupancy grid: $scene_dir/occupancy_2d/occupancy_grid.npz" >&2
  exit 1
fi
if [[ ! -f "$scene_dir/occupancy_3d/occupied_voxels_10cm.ply" ]]; then
  echo "missing 3-D occupancy voxels: $scene_dir/occupancy_3d/occupied_voxels_10cm.ply" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
python3 "$root_dir/scripts/publish_reconstructed_scene_ros2.py" "$scene_dir" &
publisher_pid=$!
cleanup() {
  kill "$publisher_pid" 2>/dev/null || true
  wait "$publisher_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

env \
  -u GTK_PATH \
  -u GIO_MODULE_DIR \
  -u GTK_MODULES \
  -u GTK_IM_MODULE_FILE \
  -u GTK_EXE_PREFIX \
  rviz2 -d "$config_path"
