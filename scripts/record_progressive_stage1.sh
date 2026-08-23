#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode_name="${1:-hm3d_stage1_progressive}"
duration="${2:-40}"
hz="${3:-10}"
idle_scan_rate="${4:-25}"
episode_dir="$root_dir/data/episodes/$episode_name"
raw_bag="$root_dir/outputs/bags/${episode_name}_raw.bag"
final_bag="$root_dir/outputs/bags/${episode_name}_final.bag"
box_dir="$root_dir/outputs/boxer/$episode_name"

for target in "$episode_dir" "$raw_bag" "$final_bag" "$box_dir"; do
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
  --duration "$duration" --hz "$hz" --follow-falcon \
  --idle-scan-rate "$idle_scan_rate" --idle-scan-after 0.3 \
  --record-dir "$episode_dir" --record-every 5
cleanup
recorder_pid=""
trap - EXIT INT TERM

./scripts/run_boxer_habitat.sh "$episode_dir" outputs/boxer
PYTHONPATH="$root_dir/third_party/boxer" .envs/boxer/bin/python \
  scripts/build_progressive_boxes.py \
  "$box_dir/boxer_3dbbs.csv" "$box_dir/boxer_3dbbs_progressive.csv"

docker compose run --rm falcon python3 \
  /workspace/falcon_ws/src/pre_map_bridge/scripts/inject_progressive_boxes.py \
  "/workspace/shared/outputs/bags/${episode_name}_raw.bag" \
  "/workspace/shared/outputs/boxer/$episode_name/boxer_3dbbs_progressive.csv" \
  "/workspace/shared/outputs/bags/${episode_name}_final.bag"

echo "最终渐进式可视化包：$final_bag"
