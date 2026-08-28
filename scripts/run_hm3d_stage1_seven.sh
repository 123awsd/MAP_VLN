#!/usr/bin/env bash
set -u -o pipefail

# Run the existing single-floor FALCON/Habitat stage-1 pipeline on the seven
# HM3D scenes used by this project.  The scene list is deliberately explicit:
# this entry point must not become a generic all-dataset launcher.

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_root="${PRE_MAP_VLN_HM3D_TRAIN_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train}"
scene_config="${PRE_MAP_VLN_HM3D_SCENE_CONFIG:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/hm3d_annotated_basis.scene_dataset_config.json}"
batch_tag="${4:-stage1_seven_v2}"
run_root="$root_dir/outputs/$batch_tag"
bag_root="$root_dir/outputs/bags"
duration="${1:-600}"
hz="${2:-10}"
record_every="${3:-5}"
scene_filter="${5:-all}"
use_planner_yaw="${6:-0}"
map_name="hm3d_stage1_seven"
map_config="$root_dir/ros_ws/src/pre_map_bridge/config/$map_name.yaml"

# Keep this order stable so batch.log is easy to compare between machines.
scene_ids=(
  "00033-oPj9qMxrDEa"
  "00062-ACZZiU6BXLz"
  "00087-YY8rqV6L6rf"
  "00108-oStKKWkQ1id"
  "00150-LcAd9dhvVwh"
  "00166-RaYrxWt5pR1"
  "00299-bdp1XNEdvmW"
)

if [[ ! -f "$scene_config" ]]; then
  echo "Habitat scene-dataset config does not exist: $scene_config" >&2
  exit 2
fi
if [[ ! -f "$map_config" ]]; then
  echo "FALCON map config does not exist: $map_config" >&2
  exit 2
fi
if ! [[ "$batch_tag" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "batch tag contains unsupported characters: $batch_tag" >&2
  exit 2
fi
if ! [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ && "$hz" =~ ^[0-9]+([.][0-9]+)?$ && "$record_every" =~ ^[1-9][0-9]*$ ]]; then
  echo "usage: $0 [duration_seconds] [hz] [record_every] [batch_tag] [scene_id|all] [use_planner_yaw:0|1]" >&2
  exit 2
fi
if [[ "$use_planner_yaw" != "0" && "$use_planner_yaw" != "1" ]]; then
  echo "use_planner_yaw must be 0 or 1" >&2
  exit 2
fi

run_scene_ids=()
if [[ "$scene_filter" == "all" ]]; then
  run_scene_ids=("${scene_ids[@]}")
else
  IFS=',' read -r -a requested_scene_ids <<< "$scene_filter"
  for requested in "${requested_scene_ids[@]}"; do
    found=0
    for known in "${scene_ids[@]}"; do
      if [[ "$requested" == "$known" ]]; then
        run_scene_ids+=("$known")
        found=1
        break
      fi
    done
    if [[ "$found" -eq 0 ]]; then
      echo "scene_filter contains a scene outside the registered seven: $requested" >&2
      exit 2
    fi
  done
fi

mkdir -p "$run_root" "$bag_root"
# The host's Docker socket is root-owned in the execution environment, as in
# the existing single-scene runner.  Keep the command explicit so a failed
# privilege check is reported before any scene is started.
docker_cmd=(sudo -n docker)

clear_bridge() {
  rm -f \
    "$root_dir/runtime/bridge/state.json" \
    "$root_dir/runtime/bridge/depth_u16.raw" \
    "$root_dir/runtime/bridge/rgb_u8.raw" \
    "$root_dir/runtime/bridge/semantic_i32.raw" \
    "$root_dir/runtime/bridge/command.json" \
    "$root_dir/runtime/bridge/exploration_complete.json" \
    "$root_dir/runtime/bridge/run_result.json"
}

run_scene() {
  local scene_id="$1"
  local token="${scene_id#*-}"
  local scene="$scene_root/$scene_id/$token.basis.glb"
  local run_dir="$run_root/$scene_id"
  local episode_dir="$run_dir/episode"
  local bag_path="$bag_root/${batch_tag}_${scene_id}_final.bag"
  local log_path="$run_dir/run.log"
  local container_name="pre-map-vln-stage1-seven-${scene_id}"
  local falcon_pid=""
  local runner_status=1
  local falcon_status=1

  echo "=== START $scene_id ==="
  if [[ ! -f "$scene" ]]; then
    echo "missing scene: $scene"
    return 1
  fi
  if [[ -e "$run_dir" || -e "$bag_path" ]]; then
    echo "skip: target already exists (no overwrite): $run_dir or $bag_path"
    return 2
  fi

  mkdir -p "$episode_dir"
  {
    printf '%s\n' \
      "scene=$scene" \
      "scene_config=$scene_config" \
      "scene_id=$scene_id" \
      "map_name=$map_name" \
      "duration_seconds=$duration" \
      "habitat_hz=$hz" \
      "record_every=$record_every" \
      "scene_filter=$scene_filter" \
      "use_planner_yaw=$use_planner_yaw" \
      "falcon_image=pre-map-vln/falcon-noetic:local" \
      "falcon_image_id=$(${docker_cmd[@]} image inspect pre-map-vln/falcon-noetic:local --format '{{.Id}}' 2>/dev/null || true)" \
      "map_config=$map_config" \
      "map_config_sha256=$(sha256sum "$map_config" | awk '{print $1}')"
  } > "$run_dir/spec.txt"
  clear_bridge

  cleanup_scene() {
    if [[ -n "$falcon_pid" ]] && kill -0 "$falcon_pid" 2>/dev/null; then
      "${docker_cmd[@]}" kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true
      wait "$falcon_pid" 2>/dev/null || true
    fi
  }
  trap cleanup_scene RETURN

  "${docker_cmd[@]}" compose run --rm --name "$container_name" falcon \
    roslaunch pre_map_bridge visualization_record_compact.launch \
    map_name:="$map_name" \
    bag_path:="/workspace/shared/outputs/bags/$(basename "$bag_path")" \
    rviz:=false topdown:=false rgb_stride:=5 \
    > "$log_path" 2>&1 &
  falcon_pid=$!

  sleep 8
  if ! kill -0 "$falcon_pid" 2>/dev/null; then
    echo "FALCON exited before Habitat start; see $log_path"
    printf '%s\n' "falcon_start_failed" > "$run_dir/status.txt"
    return 3
  fi

  set +e
  yaw_args=()
  if [[ "$use_planner_yaw" == "1" ]]; then
    yaw_args+=(--use-planner-yaw)
  fi
  "$root_dir/.envs/habitat/bin/python" "$root_dir/scripts/run_habitat_falcon.py" \
    --scene "$scene" \
    --scene-config "$scene_config" \
    --bridge-dir "$root_dir/runtime/bridge" \
    --seed 7 \
    --duration "$duration" \
    --hz "$hz" \
    --follow-falcon \
    "${yaw_args[@]}" \
    --idle-scan-rate 25 \
    --idle-scan-after 0.3 \
    --record-dir "$episode_dir" \
    --record-every "$record_every" \
    --completion-file "$root_dir/runtime/bridge/exploration_complete.json" \
    --finish-hold 3.0 \
    --result-file "$root_dir/runtime/bridge/run_result.json" \
    >> "$log_path" 2>&1
  runner_status=$?

  "${docker_cmd[@]}" kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true
  wait "$falcon_pid" 2>/dev/null
  falcon_status=$?
  set -u -o pipefail
  falcon_pid=""

  if [[ -f "$root_dir/runtime/bridge/run_result.json" ]]; then
    cp "$root_dir/runtime/bridge/run_result.json" "$run_dir/run_result.json"
  fi
  if [[ -f "$root_dir/runtime/bridge/exploration_complete.json" ]]; then
    cp "$root_dir/runtime/bridge/exploration_complete.json" "$run_dir/exploration_complete.json"
  fi
  printf '%s\n' "$runner_status" > "$run_dir/runner_exit_status.txt"
  printf '%s\n' "$falcon_status" > "$run_dir/falcon_wait_status.txt"

  if [[ "$runner_status" -eq 0 && -f "$run_dir/run_result.json" ]] \
      && jq -e '.termination == "complete"' "$run_dir/run_result.json" >/dev/null 2>&1; then
    printf '%s\n' "exploration_complete" > "$run_dir/status.txt"
    echo "=== COMPLETE $scene_id ==="
    return 0
  fi
  printf '%s\n' "exploration_failed_or_timeout" > "$run_dir/status.txt"
  echo "=== FAILED $scene_id runner=$runner_status falcon=$falcon_status (compact bag retained) ==="
  return 4
}

overall=0
for scene_id in "${run_scene_ids[@]}"; do
  run_scene "$scene_id" || overall=1
  clear_bridge
done

echo "batch_exit=$overall"
exit "$overall"
