#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
default_episodes=(Z6MFQCViBuw_01 Z6MFQCViBuw_05 Z6MFQCViBuw_08)
episodes=("${default_episodes[@]}")
if (( $# > 0 )); then
  episodes=("$@")
fi

cd "$root_dir"
for episode_id in "${episodes[@]}"; do
  prepared="$benchmark_root/runs/prepared/$episode_id"
  episode_json="$prepared/episode.json"
  scene_id="$(.envs/habitat/bin/python -c 'import json,sys; print(json.load(open(sys.argv[1]))["scene_id"])' "$episode_json")"
  category="$(.envs/habitat/bin/python -c 'import json,sys; print(json.load(open(sys.argv[1]))["category"])' "$episode_json")"

  # Full RGB is required only for the representative visualization runs.
  .envs/habitat/bin/python scripts/run_benchmark_v1.py \
    --root "$benchmark_root" --verification-mode owlv2 --only "$episode_id" \
    --rerun --save-every 1 --owlv2-every 10

  candidates="/workspace/benchmark/runs/prepared/$episode_id/candidates.json"
  if [[ "$category" == "ordered_recovery" ]]; then
    candidates="/workspace/benchmark/runs/executions/$episode_id/candidates_with_recovery.json"
  fi
  PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" docker compose run --rm falcon \
    python3 /workspace/falcon_ws/src/pre_map_bridge/scripts/build_stage2_bag.py \
    --source-bag "/workspace/benchmark/bags/${scene_id}_stage1_final.bag" \
    --execution "/workspace/benchmark/runs/executions/$episode_id/habitat_execution.json" \
    --task-graph "/workspace/benchmark/runs/prepared/$episode_id/task_graph.json" \
    --candidates "$candidates" \
    --scene-graph "/workspace/benchmark/maps/scene_graph/${scene_id}.json" \
    --frames-dir "/workspace/benchmark/runs/executions/$episode_id/frames" \
    --output-bag "/workspace/benchmark/bags/representative/${episode_id}.bag" --hz 5
done
