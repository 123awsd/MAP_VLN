#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
episode="${1:-data/episodes/hm3d_stage1}"
output_root="${2:-outputs/boxer}"

cd "$root_dir"
export PYTHONPATH="$root_dir/boxer_ext${PYTHONPATH:+:$PYTHONPATH}"
exec .envs/boxer/bin/python third_party/boxer/run_boxer.py \
  --input "$episode" \
  --labels=chair,table,sofa,bed,lamp,cabinet,door,television,plant,shelf \
  --thresh2d 0.08 \
  --thresh3d 0.2 \
  --fuse \
  --skip_viz \
  --output_dir "$output_root"
