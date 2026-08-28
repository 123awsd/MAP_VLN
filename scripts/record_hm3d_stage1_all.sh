#!/usr/bin/env bash
set -u

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_root="$root_dir/data/scene_datasets/hm3d/example"
scene_config="$scene_root/hm3d_annotated_example_basis.scene_dataset_config.json"
max_duration="${1:-600}"
hz="${2:-10}"
idle_scan_rate="${3:-25}"
postprocess="${4:-false}"
report_root="$root_dir/outputs/stage1_multiscene"
summary="$report_root/recording_summary.tsv"

mapfile -t scenes < <(find -L "$scene_root" -mindepth 2 -maxdepth 2 -type f -name '*.basis.glb' | sort)
if (( ${#scenes[@]} != 10 )); then
  echo "需要恰好 10 个 HM3D 场景，实际发现 ${#scenes[@]} 个。" >&2
  exit 1
fi

mkdir -p "$report_root"
printf 'scene\tepisode\texit_code\ttermination\n' >"$summary"
overall=0
for scene in "${scenes[@]}"; do
  scene_id="$(basename "$(dirname "$scene")")"
  episode="hm3d_stage1_${scene_id}"
  existing_result="$report_root/$scene_id/run_result.json"
  existing_raw="$root_dir/outputs/bags/${episode}_raw.bag"
  if [[ -f "$existing_raw" && -f "$existing_result" ]] && \
     [[ "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["termination"])' "$existing_result")" == "complete" ]]; then
    printf '%s\t%s\t0\tcomplete\n' "$scene_id" "$episode" >>"$summary"
    echo "===== [$scene_id] 已有完整 raw bag，跳过 ====="
    continue
  fi
  echo "===== [$scene_id] start ====="
  STAGE1_POSTPROCESS="$postprocess" "$root_dir/scripts/record_progressive_stage1.sh" \
    "$episode" "$max_duration" "$hz" "$idle_scan_rate" "$scene" "$scene_config"
  code=$?
  termination="missing_result"
  if [[ -f "$root_dir/runtime/bridge/run_result.json" ]]; then
    mkdir -p "$report_root/$scene_id"
    cp "$root_dir/runtime/bridge/run_result.json" "$report_root/$scene_id/run_result.json"
    termination="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["termination"])' "$root_dir/runtime/bridge/run_result.json")"
  fi
  printf '%s\t%s\t%s\t%s\n' "$scene_id" "$episode" "$code" "$termination" >>"$summary"
  if (( code != 0 )); then
    overall=1
  fi
  echo "===== [$scene_id] exit=$code termination=$termination ====="
done

echo "十场景录制汇总：$summary"
exit "$overall"
