#!/usr/bin/env bash
set -euo pipefail

episode_name="${1:-hm3d_panorama}"
decomp_threshold="${2:-1.5}"
exec docker compose run --rm occusg \
  bash /workspace/bridge/run_segmentation.sh "$episode_name" "$decomp_threshold"
