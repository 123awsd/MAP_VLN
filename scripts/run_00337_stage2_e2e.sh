#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"

instruction="${1:-先去一楼浴室看一下洗手池，再去二楼厨房确认微波炉。顺便看看二楼客厅沙发旁的茶几，还有三楼卧室床边的灯。最后确认三楼客厅壁炉旁的电视；如果没看到电视，就去二楼客厅找找。}"
run_name="${2:-qwen_e2e_$(date +%Y%m%d_%H%M%S)}"
verification_mode="${3:-owlv2}"
run_dir="outputs/stage2_3d/tasks/00337_${run_name}"

if [[ -e "$run_dir" ]]; then
  echo "target already exists; refusing to overwrite: $run_dir" >&2
  exit 2
fi
mkdir -p "$run_dir"

.envs/habitat/bin/python scripts/parse_stage2_instruction.py "$instruction" \
  --scene-graph outputs/stage2_3d/00337_scene_graph.json \
  --output "$run_dir/task_graph.json" \
  --budget-cny 20

exec .envs/habitat/bin/python -u scripts/run_stage2_habitat.py \
  --task-graph "$run_dir/task_graph.json" \
  --scene-graph outputs/stage2_3d/00337_scene_graph.json \
  --voxel-snapshot outputs/stage2_3d/maps/00337_dormant_recovery_clearance010_bspline_v2 \
  --generated-stage1-config outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829/generated_3d_config.json \
  --planning-config config/uav_3d_planning_habitat.yaml \
  --output-dir "$run_dir/execution" \
  --verification-mode "$verification_mode" \
  --max-candidates 8 \
  --planning-horizon-tasks 2 \
  --planning-strategy rolling_representative \
  --representatives-per-location 1 \
  --owlv2-every 10 \
  --save-every 1
