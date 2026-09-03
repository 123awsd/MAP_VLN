#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"
run_dir="$(realpath "${1:-$root_dir/outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829}")"
floors_json="$(realpath "${2:-$root_dir/outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix/floors.json}")"
floor_spacing="${3:-3.7}"
trajectory_lift="${4:-0.18}"
trajectory_width="${5:-0.10}"

to_container_path() {
  local host_path="$1"
  case "$host_path" in
    "$root_dir"/outputs/*) echo "/workspace/shared/outputs/${host_path#"$root_dir"/outputs/}" ;;
    *) echo "input must be under $root_dir/outputs: $host_path" >&2; return 1 ;;
  esac
}

cloud_path="$run_dir/bag_export/map_occupied.pcd"
trajectory_path="$run_dir/bag_export/trajectory.csv"
for required in "$cloud_path" "$trajectory_path" "$floors_json"; do
  [[ -f "$required" ]] || { echo "missing input: $required" >&2; exit 1; }
done

cd "$root_dir"
"$root_dir/scripts/prepare_rviz_xauth.sh"
pre_map_vln_resolve_docker
exec "${docker_cmd[@]}" compose run --rm --name "pre-map-vln-paper-view-${BASHPID}" falcon \
  roslaunch pre_map_bridge paper_exploration_view.launch \
  cloud_path:="$(to_container_path "$cloud_path")" \
  trajectory_path:="$(to_container_path "$trajectory_path")" \
  floors_path:="$(to_container_path "$floors_json")" \
  floor_spacing:="$floor_spacing" \
  trajectory_lift:="$trajectory_lift" \
  trajectory_width:="$trajectory_width"
