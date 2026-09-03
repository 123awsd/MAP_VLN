#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"
scene_id="${1:?usage: $0 SCENE_ID [RUN_NAME] [DURATION] [HZ] [RECORD_EVERY] [RVIZ] [RECORDER] [SEED]}"
run_name="${2:-full3d_v1}"
max_duration="${3:-600}"
hz="${4:-5}"
record_every="${5:-5}"
rviz="${6:-true}"
recorder="${7:-compact}"
seed="${8:-7}"

scene_root="${PRE_MAP_VLN_HM3D_TRAIN_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train}"
scene_config="${PRE_MAP_VLN_HM3D_SCENE_CONFIG:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/hm3d_annotated_basis.scene_dataset_config.json}"

# Development machines may keep the checked-in HM3D examples locally rather
# than mounting the full training set at /shared. Environment overrides remain
# authoritative; only replace the unavailable built-in defaults.
local_hm3d_root="$root_dir/data/versioned_data/hm3d-0.2/hm3d/example"
local_scene="$local_hm3d_root/$scene_id/${scene_id#*-}.basis.glb"
if [[ -z "${PRE_MAP_VLN_HM3D_TRAIN_ROOT:-}" && ! -f "$scene_root/$scene_id/${scene_id#*-}.basis.glb" && -f "$local_scene" ]]; then
  scene_root="$local_hm3d_root"
fi
if [[ -z "${PRE_MAP_VLN_HM3D_SCENE_CONFIG:-}" && "$scene_root" == "$local_hm3d_root" && -f "$local_hm3d_root/hm3d_annotated_basis.scene_dataset_config.json" ]]; then
  scene_config="$local_hm3d_root/hm3d_annotated_basis.scene_dataset_config.json"
fi

if [[ ! "$scene_id" =~ ^[0-9]{5}-[A-Za-z0-9]+$ ]]; then
  echo "scene id must look like 00166-RaYrxWt5pR1: $scene_id" >&2
  exit 2
fi
if [[ ! "$run_name" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "run name contains unsupported characters: $run_name" >&2
  exit 2
fi
if ! [[ "$max_duration" =~ ^[0-9]+([.][0-9]+)?$ && "$hz" =~ ^[0-9]+([.][0-9]+)?$ && "$record_every" =~ ^[1-9][0-9]*$ ]]; then
  echo "duration and hz must be positive numbers; record_every must be a positive integer" >&2
  exit 2
fi
if [[ "$rviz" != "true" && "$rviz" != "false" ]]; then
  echo "rviz must be true or false" >&2
  exit 2
fi
if [[ "$recorder" != "compact" && "$recorder" != "full" ]]; then
  echo "recorder must be compact or full" >&2
  exit 2
fi
if [[ ! "$seed" =~ ^[0-9]+$ ]]; then
  echo "seed must be a non-negative integer" >&2
  exit 2
fi

scene_number="${scene_id%%-*}"
scene_token="${scene_id#*-}"
scene="$scene_root/$scene_id/$scene_token.basis.glb"
if [[ ! -f "$scene" || ! -f "$scene_config" ]]; then
  echo "HM3D scene or dataset config is missing: $scene / $scene_config" >&2
  exit 2
fi

output_dir="$root_dir/outputs/stage1_3d/$scene_id/$run_name"
episode_dir="$output_dir/episode"
bag_name="hm3d_stage1_3d_${scene_number}_${run_name}_${recorder}.bag"
bag_path="$root_dir/outputs/bags/$bag_name"
log_path="$output_dir/run.log"
generated_yaml="$output_dir/generated_3d_config.yaml"
generated_json="$output_dir/generated_3d_config.json"
map_name="hm3d_${scene_number}_3d_generated"
container_name="pre-map-vln-3d-${scene_number}-${run_name}"
bridge_dir="$root_dir/runtime/bridge"
pre_map_vln_resolve_docker

for target in "$output_dir" "$bag_path"; do
  if [[ -e "$target" ]]; then
    echo "target already exists; refusing to overwrite: $target" >&2
    exit 2
  fi
done

mkdir -p "$episode_dir" "$root_dir/outputs/bags"
"$root_dir/.envs/habitat/bin/python" "$root_dir/scripts/generate_hm3d_3d_config.py" \
  --scene "$scene" \
  --scene-config "$scene_config" \
  --map-name "$map_name" \
  --seed "$seed" \
  --output-yaml "$generated_yaml" \
  --output-json "$generated_json"

read -r initial_x initial_y initial_z < <(
  "$root_dir/.envs/habitat/bin/python" -c \
    'import json,sys; p=json.load(open(sys.argv[1])); print(*p["initial_agent_habitat_xyz"])' \
    "$generated_json"
)

rm -f \
  "$bridge_dir/state.json" \
  "$bridge_dir/depth_u16.raw" \
  "$bridge_dir/rgb_u8.raw" \
  "$bridge_dir/semantic_i32.raw" \
  "$bridge_dir/command.json" \
  "$bridge_dir/exploration_complete.json" \
  "$bridge_dir/run_result.json"

printf '%s\n' \
  "scene=$scene" \
  "scene_config=$scene_config" \
  "scene_id=$scene_id" \
  "map_name=$map_name" \
  "generated_map_config=$generated_yaml" \
  "run_name=$run_name" \
  "max_duration_seconds=$max_duration" \
  "habitat_hz=$hz" \
  "record_every=$record_every" \
  "recorder=$recorder" \
  "seed=$seed" \
  "initial_agent_habitat_xyz=$initial_x $initial_y $initial_z" \
  "use_planner_yaw=true" \
  "allow_vertical_motion=true" \
  "falcon_image=pre-map-vln/falcon-noetic:local" \
  "falcon_image_id=$(${docker_cmd[@]} image inspect pre-map-vln/falcon-noetic:local --format '{{.Id}}')" \
  > "$output_dir/spec.txt"

falcon_pid=""
cleanup() {
  if [[ -n "$falcon_pid" ]] && kill -0 "$falcon_pid" 2>/dev/null; then
    "${docker_cmd[@]}" kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true
    wait "$falcon_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

container_config="/workspace/shared/outputs/stage1_3d/$scene_id/$run_name/$(basename "$generated_yaml")"
record_launch="visualization_record_compact.launch"
record_args=(rviz:="$rviz" topdown:=false)
if [[ "$recorder" == "full" ]]; then
  record_launch="visualization_record.launch"
else
  record_args+=(rgb_stride:=5)
fi
"${docker_cmd[@]}" compose run --rm --name "$container_name" falcon \
  roslaunch pre_map_bridge "$record_launch" \
  map_name:="$map_name" \
  map_config:="$container_config" \
  bag_path:="/workspace/shared/outputs/bags/$bag_name" \
  "${record_args[@]}" \
  > "$log_path" 2>&1 &
falcon_pid=$!

sleep 8
if ! kill -0 "$falcon_pid" 2>/dev/null; then
  echo "FALCON exited before Habitat started; see $log_path" >&2
  exit 3
fi

set +e
"$root_dir/.envs/habitat/bin/python" "$root_dir/scripts/run_habitat_falcon.py" \
  --scene "$scene" \
  --scene-config "$scene_config" \
  --bridge-dir "$bridge_dir" \
  --seed "$seed" \
  --initial-position "$initial_x" "$initial_y" "$initial_z" \
  --duration "$max_duration" \
  --hz "$hz" \
  --follow-falcon \
  --allow-vertical-motion \
  --use-planner-yaw \
  --idle-scan-rate 25 \
  --idle-scan-after 0.3 \
  --record-dir "$episode_dir" \
  --record-every "$record_every" \
  --completion-file "$bridge_dir/exploration_complete.json" \
  --finish-hold 3.0 \
  --result-file "$bridge_dir/run_result.json" \
  >> "$log_path" 2>&1
runner_status=$?
set -e

"${docker_cmd[@]}" kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true
set +e
wait "$falcon_pid" 2>/dev/null
falcon_status=$?
set -e
falcon_pid=""
trap - EXIT INT TERM

if [[ -f "$bridge_dir/run_result.json" ]]; then
  cp "$bridge_dir/run_result.json" "$output_dir/run_result.json"
fi
if [[ -f "$bridge_dir/exploration_complete.json" ]]; then
  cp "$bridge_dir/exploration_complete.json" "$output_dir/exploration_complete.json"
fi
printf '%s\n' "$runner_status" > "$output_dir/runner_exit_status.txt"
printf '%s\n' "$falcon_status" > "$output_dir/falcon_wait_status.txt"

# Keep a scene-centric, zero-copy view of runs and Bags for browsing.  The
# historical artifact paths remain canonical so existing replay/postprocess
# commands continue to work.
"$root_dir/.envs/habitat/bin/python" "$root_dir/scripts/refresh_scene_output_index.py" \
  --scene-id "$scene_id" || \
  echo "warning: unable to refresh outputs/scenes index" >&2

if [[ "$runner_status" -ne 0 ]]; then
  echo "Habitat runner failed with exit $runner_status; $recorder Bag retained: $bag_path" >&2
  exit "$runner_status"
fi
if [[ ! -f "$output_dir/run_result.json" ]] || ! jq -e '.termination == "complete"' "$output_dir/run_result.json" >/dev/null; then
  echo "run did not enter legal FINISH; inspect $log_path and $output_dir/run_result.json" >&2
  exit 3
fi

echo "3-D exploration entered legal FINISH: $output_dir"
echo "$recorder Bag: $bag_path"
