#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode="${1:-data/episodes/hm3d_stage1}"
output_root="${2:-outputs/boxer}"
vocabulary="${3:-config/stage1_indoor_v2.txt}"
write_name="${4:-boxer}"
detector="${5:-grounding_dino}"
segmentation="${6:-sam2}"
max_frames="${7:-99999}"
skip_frames="${8:-1}"
visualization="${9:-false}"
detection_threshold="${10:-}"
if [[ -z "$detection_threshold" ]]; then
  if [[ "$detector" == "grounding_dino" ]]; then
    detection_threshold="0.30"
  else
    detection_threshold="0.20"
  fi
fi

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
if [[ "$detector" != "owl" && "$detector" != "grounding_dino" ]]; then
  echo "不支持的 2D detector：$detector" >&2
  exit 1
fi
if [[ "$segmentation" != "none" && "$segmentation" != "sam2" ]]; then
  echo "不支持的分割方式：$segmentation" >&2
  exit 1
fi
if [[ "$segmentation" == "sam2" && "$detector" != "grounding_dino" ]]; then
  echo "SAM2 深度过滤当前需要 grounding_dino detector" >&2
  exit 1
fi
extra_detector_args=(--detector "$detector")
if [[ "$segmentation" == "sam2" ]]; then
  extra_detector_args+=(--sam2)
fi
visualization_args=(--skip_viz)
if [[ "$visualization" == "true" ]]; then
  visualization_args=()
fi
echo "Boxer 离线词表：$vocabulary（${#labels[@]} 类），2D=$detector，mask=$segmentation，threshold=$detection_threshold，输出前缀：$write_name"
export PYTHONPATH="$root_dir/boxer_ext${PYTHONPATH:+:$PYTHONPATH}"
.envs/boxer/bin/python third_party/boxer/run_boxer.py \
  --input "$episode" \
  --labels="$labels_csv" \
  "${extra_detector_args[@]}" \
  --thresh2d "$detection_threshold" \
  --thresh3d 0.2 \
  --max_n "$max_frames" \
  --skip_n "$skip_frames" \
  --write_name "$write_name" \
  "${visualization_args[@]}" \
  --output_dir "$output_root"

sequence_name="$(basename "${episode%/}")"
raw_csv="$output_root/$sequence_name/${write_name}_3dbbs.csv"
fused_csv="$output_root/$sequence_name/${write_name}_3dbbs_fused.csv"
PYTHONPATH="$root_dir/third_party/boxer" .envs/boxer/bin/python \
  third_party/boxer/utils/fuse_3d_boxes.py \
  --input "$raw_csv" --output "$fused_csv" \
  --iou 0.2 --min_detections 4 --conf_threshold 0.45
