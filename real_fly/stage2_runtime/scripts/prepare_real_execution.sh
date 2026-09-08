#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 RUN_ID /path/to/mission_plan.json [MISSION_ID]" >&2
  exit 2
fi
run_id="$1"
mission="$2"
mission_id="${3:-}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
runtime_root="$(cd "$script_dir/.." && pwd)"
root="$(cd "$runtime_root/../.." && pwd)"
run_data="$root/real_fly/stage2_offline/data/$run_id"
map_pcd="$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd"
metadata="$run_data/voxel_snapshot/metadata.json"
if [[ ! -r "$metadata" && -r "$run_data/voxel_snapshot_complete/metadata.json" ]]; then
  metadata="$run_data/voxel_snapshot_complete/metadata.json"
fi
if [[ -n "$mission_id" ]]; then
  [[ "$mission_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || {
    echo "Invalid MISSION_ID: $mission_id" >&2
    exit 2
  }
  output="$runtime_root/missions/$run_id/$mission_id/execution_bundle.json"
else
  output="$runtime_root/missions/$run_id/execution_bundle.json"
fi

exec python3 "$script_dir/prepare_execution_bundle.py" \
  --mission "$mission" --map "$map_pcd" --voxel-metadata "$metadata" \
  --output "$output"
