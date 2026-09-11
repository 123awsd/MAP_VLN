#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"

task_name="${1:-main}"
run_name="${2:-${task_name}_$(date +%Y%m%d_%H%M%S)}"
verification_mode="${3:-owlv2}"
recovery_mode="${4:-qwen}"
branch_mode="${5:-normal}"

case "$task_name" in
  main)
    task_graph="config/stage2_tasks/00337_household_multifloor.json"
    extra_args=()
    ;;
  recovery)
    task_graph="config/stage2_tasks/00337_toilet_paper_recovery.json"
    extra_args=(
      --semantic-recovery
      --controlled-stale-object-ids '["L2_boxer_50"]'
      --max-recovery-visits 8
      --max-viewpoints-per-location 2
    )
    if [[ "$recovery_mode" == "offline" ]]; then
      extra_args+=(--recovery-plan config/stage2_tasks/00337_toilet_paper_offline_recovery_plan.json)
    elif [[ "$recovery_mode" != "qwen" ]]; then
      echo "recovery mode must be qwen or offline" >&2
      exit 2
    fi
    ;;
  *)
    echo "task must be main or recovery" >&2
    exit 2
    ;;
esac

output_dir="outputs/stage2_3d/tasks/00337_${run_name}"
if [[ -e "$output_dir" ]]; then
  echo "target already exists; refusing to overwrite: $output_dir" >&2
  exit 2
fi

controlled_args=()
if [[ "$verification_mode" == "controlled" && "$task_name" == "main" ]]; then
  controlled_args=(--controlled-found-object-ids '["L1_boxer_0","L2_boxer_32","L2_boxer_58","L3_boxer_22","L3_boxer_25","L2_boxer_54","L2_boxer_62"]')
  if [[ "$branch_mode" == "tv_not_found" ]]; then
    controlled_args+=(--controlled-stale-object-ids '["L3_boxer_25"]')
  elif [[ "$branch_mode" != "normal" ]]; then
    echo "branch mode must be normal or tv_not_found" >&2
    exit 2
  fi
fi

exec .envs/habitat/bin/python -u scripts/run_stage2_habitat.py \
  --task-graph "$task_graph" \
  --scene-graph outputs/stage2_3d/00337_scene_graph.json \
  --voxel-snapshot outputs/stage2_3d/maps/00337_dormant_recovery_clearance010_bspline_v2 \
  --generated-stage1-config outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829/generated_3d_config.json \
  --planning-config config/uav_3d_planning_habitat.yaml \
  --output-dir "$output_dir" \
  --verification-mode "$verification_mode" \
  --max-candidates 3 \
  --planning-horizon-tasks 2 \
  --planning-strategy rolling_representative \
  --representatives-per-location 1 \
  --owlv2-every 10 \
  --save-every 5 \
  "${extra_args[@]}" \
  "${controlled_args[@]}"
