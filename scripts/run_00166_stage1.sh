#!/usr/bin/env bash
set -euo pipefail

# One-scene experiment entry point.  It intentionally hard-codes the supplied
# 00166 assets so an accidental run on another HM3D map is rejected here.
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"
scene_root="${PRE_MAP_VLN_HM3D_TRAIN_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train}"
scene_config="${PRE_MAP_VLN_HM3D_SCENE_CONFIG:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/hm3d_annotated_basis.scene_dataset_config.json}"
scene="$scene_root/00166-RaYrxWt5pR1/RaYrxWt5pR1.basis.glb"
map_name="${5:-hm3d_00166}"

run_name="${1:-ground_v1}"
max_duration="${2:-600}"
hz="${3:-10}"
record_every="${4:-5}"

case "$map_name" in
  hm3d_00166|hm3d_00166_v2|hm3d_00166_v3|hm3d_00166_v4|hm3d_00166_v5|hm3d_00166_v6) ;;
  *)
    echo "只允许 00166 的已登记配置名：hm3d_00166、hm3d_00166_v2、hm3d_00166_v3、hm3d_00166_v4、hm3d_00166_v5 或 hm3d_00166_v6；收到 $map_name" >&2
    exit 2
    ;;
esac

if [[ ! "$run_name" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "实验名只能包含字母、数字、点、下划线和连字符：$run_name" >&2
  exit 2
fi
if [[ ! -f "$scene" || ! -f "$scene_config" ]]; then
  echo "00166 Habitat 资产不存在：$scene 或 $scene_config" >&2
  exit 2
fi
map_config="$root_dir/ros_ws/src/pre_map_bridge/config/$map_name.yaml"
if [[ ! -f "$map_config" ]]; then
  echo "00166 FALCON 配置不存在：$map_config" >&2
  exit 2
fi

output_dir="$root_dir/outputs/stage1_00166/$run_name"
episode_dir="$output_dir/episode"
bag_path="$root_dir/outputs/bags/hm3d_stage1_00166_${run_name}_raw.bag"
log_path="$output_dir/run.log"
container_name="pre-map-vln-00166-${run_name}"

for target in "$output_dir" "$bag_path"; do
  if [[ -e "$target" ]]; then
    echo "目标已存在，拒绝覆盖：$target" >&2
    exit 2
  fi
done

pre_map_vln_resolve_docker
mkdir -p "$episode_dir" "$root_dir/outputs/bags"
printf '%s\n' \
  "scene=$scene" \
  "scene_config=$scene_config" \
  "map_name=$map_name" \
  "run_name=$run_name" \
  "max_duration_seconds=$max_duration" \
  "habitat_hz=$hz" \
  "record_every=$record_every" \
  "falcon_image=pre-map-vln/falcon-noetic:local" \
  "map_config=$map_config" \
  "map_config_sha256=$(sha256sum "$map_config" | awk '{print $1}')" \
  "falcon_image_id=$(${docker_cmd[@]} image inspect pre-map-vln/falcon-noetic:local --format '{{.Id}}')" > "$output_dir/spec.txt"

# These are transient synchronization files, not experiment results.  The
# explicit list prevents a previous episode's last frame/FINISH flag from
# contaminating this run.
rm -f \
  "$root_dir/runtime/bridge/state.json" \
  "$root_dir/runtime/bridge/depth_u16.raw" \
  "$root_dir/runtime/bridge/rgb_u8.raw" \
  "$root_dir/runtime/bridge/semantic_i32.raw" \
  "$root_dir/runtime/bridge/command.json" \
  "$root_dir/runtime/bridge/exploration_complete.json" \
  "$root_dir/runtime/bridge/run_result.json"

falcon_pid=""
cleanup() {
  if [[ -n "$falcon_pid" ]] && kill -0 "$falcon_pid" 2>/dev/null; then
    "${docker_cmd[@]}" kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true
    wait "$falcon_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"${docker_cmd[@]}" compose run --rm --name "$container_name" falcon \
  roslaunch pre_map_bridge visualization_record.launch \
  map_name:="$map_name" \
  bag_path:="/workspace/shared/outputs/bags/$(basename "$bag_path")" \
  rviz:=false topdown:=false \
  > "$log_path" 2>&1 &
falcon_pid=$!

# Give roslaunch enough time to start the master, mapper and recorder.  The
# runner itself waits for the bridge command through its normal polling loop.
sleep 8
if ! kill -0 "$falcon_pid" 2>/dev/null; then
  echo "FALCON 容器在 Habitat 启动前已退出，详见 $log_path" >&2
  exit 1
fi

set +e
"$root_dir/.envs/habitat/bin/python" "$root_dir/scripts/run_habitat_falcon.py" \
  --scene "$scene" \
  --scene-config "$scene_config" \
  --bridge-dir "$root_dir/runtime/bridge" \
  --seed 7 \
  --duration "$max_duration" \
  --hz "$hz" \
  --follow-falcon \
  --idle-scan-rate 25 \
  --idle-scan-after 0.3 \
  --record-dir "$episode_dir" \
  --record-every "$record_every" \
  --completion-file "$root_dir/runtime/bridge/exploration_complete.json" \
  --finish-hold 3.0 \
  --result-file "$root_dir/runtime/bridge/run_result.json" \
  >> "$log_path" 2>&1
runner_status=$?
set -e

"${docker_cmd[@]}" kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true
set +e
wait "$falcon_pid" 2>/dev/null
falcon_status=$?
set -e
falcon_pid=""

if [[ -f "$root_dir/runtime/bridge/run_result.json" ]]; then
  cp "$root_dir/runtime/bridge/run_result.json" "$output_dir/run_result.json"
fi
if [[ -f "$root_dir/runtime/bridge/exploration_complete.json" ]]; then
  cp "$root_dir/runtime/bridge/exploration_complete.json" "$output_dir/exploration_complete.json"
fi
printf '%s\n' "$runner_status" > "$output_dir/runner_exit_status.txt"
printf '%s\n' "$falcon_status" > "$output_dir/falcon_wait_status.txt"

if [[ "$runner_status" -ne 0 ]]; then
  echo "00166 实验失败：Habitat runner exit=$runner_status；raw bag 保留为失败证据：$bag_path" >&2
  exit "$runner_status"
fi
if [[ ! -f "$output_dir/run_result.json" ]] || ! jq -e '.termination == "complete"' "$output_dir/run_result.json" >/dev/null; then
  echo "00166 实验失败：未进入合法 FINISH；raw bag 保留为失败证据：$bag_path" >&2
  exit 3
fi

echo "00166 实验完成信号已检测到；尚需通过真值覆盖率和规划/碰撞分析：$output_dir"
