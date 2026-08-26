#!/usr/bin/env bash
set -uo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
maps_exit="$benchmark_root/runs/maps_phase.exit_code"
export_exit="$benchmark_root/runs/roofless_export.exit_code"

echo "waiting for maps phase completion: $maps_exit"
while [[ ! -f "$maps_exit" ]]; do
  sleep 30
done
echo "maps phase ended with exit_code=$(<"$maps_exit"); exporting every available final scene"

"$root_dir/scripts/export_benchmark_roofless_images.sh"
code=$?
printf '%s\n' "$code" >"$export_exit"
exit "$code"
