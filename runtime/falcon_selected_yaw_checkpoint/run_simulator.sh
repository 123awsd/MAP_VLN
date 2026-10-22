#!/usr/bin/env bash
# Restored baseline: fresh output required; never overwrite archived runs.
set -euo pipefail
trial_dir="${1:?Pass a freshly prepared trial directory}"
duration="${2:-504}"
hz="${3:-5}"
trial_dir="$(readlink -f "$trial_dir")"
cd /home/uav/map_VLN
[[ -d "$trial_dir/bridge" && -f "$trial_dir/config.yaml" ]] || exit 2
[[ ! -e "$trial_dir/result.json" && ! -e "$trial_dir/bridge/state.json" ]] || {
 echo 'Existing observations found; prepare a fresh trial directory.' >&2
 exit 2
}
exec PRE_MAP_VLN/.envs/habitat/bin/python PRE_MAP_VLN/scripts/run_habitat_falcon.py \
 --scene PRE_MAP_VLN/data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb \
 --scene-config PRE_MAP_VLN/data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json \
 --bridge-dir "$trial_dir/bridge" \
 --seed 7 --initial-position 9.714411735534668 -2.6005380153656006 -2.374816417694092 \
 --duration "$duration" --hz "$hz" --follow-falcon --allow-vertical-motion --use-planner-yaw --synchronized-frames \
 --scene-collision-check --collision-radius 0.01 \
 --scene-repair-config PRE_MAP_VLN/scene_repairs/00337_three_floor_shell/repair.json \
 --completion-file "$trial_dir/bridge/exploration_complete.json" \
 --result-file "$trial_dir/result.json"
