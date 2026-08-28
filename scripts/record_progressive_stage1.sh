#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode_name="${1:-hm3d_stage1_progressive}"
max_duration="${2:-600}"
hz="${3:-10}"
idle_scan_rate="${4:-25}"
scene="${5:-$root_dir/data/scene_datasets/hm3d/example/00861-GLAQ4DNUx5U/GLAQ4DNUx5U.basis.glb}"
scene_config="${6:-$root_dir/data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json}"
postprocess="${STAGE1_POSTPROCESS:-true}"
episode_dir="$root_dir/data/episodes/$episode_name"
raw_bag="$root_dir/outputs/bags/${episode_name}_raw.bag"
final_bag="$root_dir/outputs/bags/${episode_name}_final.bag"
box_dir="$root_dir/outputs/boxer/$episode_name"

targets=("$episode_dir" "$raw_bag")
if [[ "$postprocess" == "true" ]]; then
  targets+=("$final_bag" "$box_dir")
fi
for target in "${targets[@]}"; do
  if [[ -e "$target" ]]; then
    echo "目标已存在，拒绝覆盖：$target" >&2
    exit 1
  fi
done

cd "$root_dir"
mkdir -p "$episode_dir" outputs/bags
recorder_pid=""
cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    docker kill --signal=SIGINT pre-map-vln-falcon-vis >/dev/null 2>&1 || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

./scripts/run_falcon_record.sh "${episode_name}_raw" false &
recorder_pid=$!
sleep 3
.envs/habitat/bin/python scripts/run_habitat_falcon.py \
  --scene "$scene" --scene-config "$scene_config" \
  --duration "$max_duration" --hz "$hz" --follow-falcon \
  --idle-scan-rate "$idle_scan_rate" --idle-scan-after 0.3 \
  --record-dir "$episode_dir" --record-every 5 \
  --completion-file "$root_dir/runtime/bridge/exploration_complete.json" \
  --finish-hold 3.0 \
  --result-file "$root_dir/runtime/bridge/run_result.json"
cleanup
recorder_pid=""
trap - EXIT INT TERM

termination="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["termination"])' \
  "$root_dir/runtime/bridge/run_result.json")"
if [[ "$termination" != "complete" ]]; then
  echo "达到最大时长 ${max_duration}s，但 FALCON 未进入 FINISH；保留 raw bag，拒绝生成 complete 包。" >&2
  exit 2
fi

if [[ "$postprocess" != "true" ]]; then
  echo "探索建图录制完成（未执行后处理）：$raw_bag"
  exit 0
fi

./scripts/run_boxer_habitat.sh "$episode_dir" outputs/boxer
PYTHONPATH="$root_dir/third_party/boxer" .envs/boxer/bin/python \
  scripts/build_progressive_boxes.py \
  "$box_dir/boxer_3dbbs.csv" "$box_dir/boxer_3dbbs_progressive.csv" \
  --iou 0.2 --min-detections 4 --conf-threshold 0.45

docker compose run --rm falcon python3 \
  /workspace/falcon_ws/src/pre_map_bridge/scripts/inject_progressive_boxes.py \
  "/workspace/shared/outputs/bags/${episode_name}_raw.bag" \
  "/workspace/shared/outputs/boxer/$episode_name/boxer_3dbbs_progressive.csv" \
  "/workspace/shared/outputs/bags/${episode_name}_final.bag"

echo "最终渐进式可视化包：$final_bag"
