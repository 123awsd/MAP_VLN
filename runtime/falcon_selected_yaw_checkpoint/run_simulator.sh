#!/usr/bin/env bash
set -euo pipefail
cd /home/uav/map_VLN
PRE_MAP_VLN/.envs/habitat/bin/python PRE_MAP_VLN/scripts/run_habitat_falcon.py \
 --scene PRE_MAP_VLN/data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb \
 --scene-config PRE_MAP_VLN/data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json \
 --bridge-dir PRE_MAP_VLN/outputs/exploration_yaw_consistent_trial/bridge \
 --seed 7 --initial-position 9.714411735534668 -2.6005380153656006 -2.374816417694092 \
 --duration 720 --hz 5 --follow-falcon --allow-vertical-motion --use-planner-yaw --synchronized-frames \
 --scene-collision-check \
 --scene-repair-config PRE_MAP_VLN/scene_repairs/00337_three_floor_shell/repair.json \
 --completion-file PRE_MAP_VLN/outputs/exploration_yaw_consistent_trial/bridge/exploration_complete.json \
 --result-file PRE_MAP_VLN/outputs/exploration_yaw_consistent_trial/result.json
