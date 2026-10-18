#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode="${1:-data/episodes/hm3d_stage1}"
output_root="${2:-outputs/boxer}"
vocabulary="${3:-config/boxer_indoor_30.txt}"
write_name="${4:-boxer}"
detector="${PRE_MAP_VLN_BOXER_DETECTOR:-sam31}"
case "$detector" in
  sam31)
    boxer_python="$root_dir/.envs/sam31/bin/python"
    threshold2d="0.50"
    detector_args=(--sam31_checkpoint "$root_dir/models/sam31/sam3.1_multiplex.safetensors")
    ;;
  owl|grounding_dino)
    boxer_python="$root_dir/.envs/boxer/bin/python"
    threshold2d="0.25"
    detector_args=()
    ;;
  *) echo "未知检测器：$detector" >&2; exit 2 ;;
esac
[[ -x "$boxer_python" ]] || { echo "缺少运行环境：$boxer_python" >&2; exit 2; }

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
echo "Boxer 离线词表：$vocabulary（${#labels[@]} 类），输出前缀：$write_name，前端：$detector"
export PYTHONPATH="$root_dir/boxer_ext${PYTHONPATH:+:$PYTHONPATH}"
"$boxer_python" scripts/run_seeded_boxer.py \
  --detector "$detector" "${detector_args[@]}" \
  --input "$episode" \
  --labels="$labels_csv" \
  --thresh2d "$threshold2d" \
  --thresh3d 0.2 \
  --write_name "$write_name" \
  --skip_viz \
  --output_dir "$output_root"

sequence_name="$(basename "${episode%/}")"
raw_csv="$output_root/$sequence_name/${write_name}_3dbbs.csv"
fused_csv="$output_root/$sequence_name/${write_name}_3dbbs_fused.csv"
PYTHONPATH="$root_dir/third_party/boxer" "$boxer_python" \
  third_party/boxer/utils/fuse_3d_boxes.py \
  --input "$raw_csv" --output "$fused_csv" \
  --iou 0.2 --min_detections 4 --conf_threshold 0.45
