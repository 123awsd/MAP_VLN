#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
open_nav="$project_root/third_party/Open-Nav"

mkdir -p "$open_nav/waypoint_prediction/checkpoints"
mkdir -p "$open_nav/data/pretrained_models/ddppo-models"
mkdir -p "$project_root/data/baselines"
mkdir -p "$open_nav/data/scene_datasets"
mkdir -p "$project_root/third_party/VLN-Zero/VLN_CE/data/scene_datasets"

ln -sfn \
  "$project_root/checkpoints/baselines/opennav/check_val_best_avg_wayscore" \
  "$open_nav/waypoint_prediction/checkpoints/check_val_best_avg_wayscore"
ln -sfn \
  "$project_root/checkpoints/baselines/opennav/gibson-2plus-resnet50.pth" \
  "$open_nav/data/pretrained_models/ddppo-models/gibson-2plus-resnet50.pth"
ln -sfn \
  "$open_nav/data/datasets/R2R_VLNCE_v1-2_preprocessed/val_unseen" \
  "$project_root/data/baselines/opennav_r2r_ce_100"
ln -sfn \
  "$project_root/data/scene_datasets/mp3d" \
  "$open_nav/data/scene_datasets/mp3d"
ln -sfn \
  "$project_root/data/scene_datasets/mp3d" \
  "$project_root/third_party/VLN-Zero/VLN_CE/data/scene_datasets/mp3d"

echo "External baseline assets linked without copying large files."
