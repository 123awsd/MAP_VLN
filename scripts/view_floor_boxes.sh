#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${1:?usage: view_floor_boxes.sh RUN_DIR BOXES_CSV [SELECTED_FLOOR] [MAX_FLOOR]}"
boxes_csv="${2:?usage: view_floor_boxes.sh RUN_DIR BOXES_CSV [SELECTED_FLOOR] [MAX_FLOOR]}"
selected_floor="${3:-0}"
max_floor="${4:-0}"

run_dir="$(realpath "$run_dir")"
boxes_csv="$(realpath "$boxes_csv")"
cloud_path="$run_dir/bag_export/map_occupied.pcd"
trajectory_path="$run_dir/bag_export/trajectory.csv"

for required in "$cloud_path" "$trajectory_path" "$boxes_csv"; do
  if [[ ! -f "$required" ]]; then
    echo "required floor-view input is missing: $required" >&2
    exit 1
  fi
done

to_container_path() {
  local host_path="$1"
  case "$host_path" in
    "$root_dir"/outputs/*)
      echo "/workspace/shared/outputs/${host_path#"$root_dir"/outputs/}"
      ;;
    *)
      echo "floor-view inputs must be under $root_dir/outputs: $host_path" >&2
      return 1
      ;;
  esac
}

cloud_container="$(to_container_path "$cloud_path")"
boxes_container="$(to_container_path "$boxes_csv")"
trajectory_container="$(to_container_path "$trajectory_path")"

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
exec docker compose run --rm --name "pre-map-vln-floor-view-${BASHPID}" falcon \
  roslaunch pre_map_bridge floor_layer_view.launch \
  cloud_path:="$cloud_container" \
  boxes_path:="$boxes_container" \
  trajectory_path:="$trajectory_container" \
  floor_detection_mode:=falcon \
  selected_floor:="$selected_floor" \
  max_floor:="$max_floor"
