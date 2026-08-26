#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode="${1:-data/episodes/hm3d_stage1}"
output_root="${2:-outputs/boxer}"
vocabulary="${3:-config/stage1_indoor_full_v1.txt}"
write_name="${4:-boxer}"

cd "$root_dir"
if [[ ! -f "$vocabulary" ]]; then
  echo "词表不存在：$vocabulary" >&2
  exit 1
fi
mapfile -t labels < <(awk 'NF && $1 !~ /^#/ {sub(/^[[:space:]]+/, ""); sub(/[[:space:]]+$/, ""); print}' "$vocabulary")
if [[ "${#labels[@]}" -eq 0 ]]; then
  echo "词表为空：$vocabulary" >&2
  exit 1
fi
labels_csv="$(IFS=,; echo "${labels[*]}")"
echo "Boxer 离线词表：$vocabulary（${#labels[@]} 类），输出前缀：$write_name"
export PYTHONPATH="$root_dir/boxer_ext${PYTHONPATH:+:$PYTHONPATH}"
.envs/boxer/bin/python third_party/boxer/run_boxer.py \
  --input "$episode" \
  --labels="$labels_csv" \
  --thresh2d 0.20 \
  --thresh3d 0.2 \
  --write_name "$write_name" \
  --skip_viz \
  --output_dir "$output_root"

sequence_name="$(basename "${episode%/}")"
raw_csv="$output_root/$sequence_name/${write_name}_3dbbs.csv"
fused_csv="$output_root/$sequence_name/${write_name}_3dbbs_fused.csv"
PYTHONPATH="$root_dir/third_party/boxer" .envs/boxer/bin/python \
  third_party/boxer/utils/fuse_3d_boxes.py \
  --input "$raw_csv" --output "$fused_csv" \
  --iou 0.2 --min_detections 4 --conf_threshold 0.45
