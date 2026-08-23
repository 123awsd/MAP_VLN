#!/usr/bin/env bash
set -euo pipefail

episode_name="${1:-hm3d_panorama}"
exec docker compose run --rm occusg \
  bash /workspace/bridge/run_segmentation.sh "$episode_name"
