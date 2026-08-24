#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
open_nav="$project_root/third_party/Open-Nav"

cd "$open_nav"
exec "$project_root/scripts/run_vlnce_legacy.sh" open-nav run.py \
  --exp_name pre_map_vln_smoke \
  --exp-config run_OpenNav.yaml \
  --llm qwen3.7-plus \
  EVAL.EPISODE_COUNT 1 \
  TASK_CONFIG.DATASET.EPISODES_TO_LOAD 1 \
  RESULTS_DIR "$project_root/outputs/baselines/opennav/"
