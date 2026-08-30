#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"

instruction='去一楼卫生间的洗手台旁帮我找一卷卫生纸，找到后拍张照片确认一下。'
run_name="${1:-toilet_paper_real_recovery_$(date +%Y%m%d_%H%M%S)}"
run_dir="outputs/stage2_3d/tasks/00337_${run_name}"
bag_name="00337_${run_name}_rviz"
source_bag="outputs/bags/hm3d_stage1_3d_00337_dormant_recovery_20260829_compact.bag"

if [[ -e "$run_dir" ]]; then
  echo "target already exists; refusing to overwrite: $run_dir" >&2
  exit 2
fi
mkdir -p "$run_dir"

echo "[1/3] Qwen parses the explicit-location search task"
.envs/habitat/bin/python scripts/parse_stage2_instruction.py "$instruction" \
  --scene-graph outputs/stage2_3d/00337_scene_graph.json \
  --output "$run_dir/task_graph.json" \
  --budget-cny 20

echo "[2/3] Habitat executes strict local views, then Qwen recovery if exhausted"
.envs/habitat/bin/python -u scripts/run_stage2_habitat.py \
  --task-graph "$run_dir/task_graph.json" \
  --scene-graph outputs/stage2_3d/00337_scene_graph.json \
  --voxel-snapshot outputs/stage2_3d/maps/00337_dormant_recovery_clearance010_bspline_v2 \
  --generated-stage1-config outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829/generated_3d_config.json \
  --planning-config config/uav_3d_planning_habitat.yaml \
  --output-dir "$run_dir/execution" \
  --verification-mode owlv2 \
  --owlv2-class-thresholds '{"toilet paper":0.35}' \
  --semantic-recovery \
  --max-candidates 4 \
  --max-viewpoints-per-location 3 \
  --max-recovery-hypotheses 3 \
  --max-recovery-locations 4 \
  --planning-horizon-tasks 1 \
  --owlv2-every 10 \
  --save-every 1

echo "[3/3] Build the RViz replay bag"
./scripts/build_stage2_rviz_replay.sh \
  "$run_dir" "$source_bag" "$bag_name"

echo
echo "run_dir=$root_dir/$run_dir"
echo "bag=$root_dir/outputs/bags/${bag_name}.bag"
echo "Replay with:"
echo "./scripts/replay_stage2_rviz.sh $bag_name false 1.0 $run_dir/execution/updated_scene_graph.json"
