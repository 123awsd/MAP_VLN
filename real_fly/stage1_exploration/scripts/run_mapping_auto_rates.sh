#!/usr/bin/env bash
# Try complete offline FAST-LIO at progressively safer rosbag rates.
set -euo pipefail

usage() {
  echo "Usage: $0 --bag FILE --run-id ID --runtime-root DIR --output-dir DIR [--rates '1.0 0.5 0.25 0.125'] [--master-port PORT]"
}

bag=""
run_id=""
runtime_root=""
output_dir=""
rates="1.0 0.5 0.25 0.125"
port=11312
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) bag="$2"; shift 2 ;;
    --run-id) run_id="$2"; shift 2 ;;
    --runtime-root) runtime_root="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    --rates) rates="$2"; shift 2 ;;
    --master-port) port="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ -n "$bag" && -n "$run_id" && -n "$runtime_root" && -n "$output_dir" ]] || {
  usage >&2; exit 2;
}
[[ ! -e "$output_dir" ]] || { echo "Refusing to overwrite: $output_dir" >&2; exit 2; }
attempt_root="${output_dir}.attempts"
[[ ! -e "$attempt_root" ]] || { echo "Refusing to overwrite prior attempts: $attempt_root" >&2; exit 2; }
mkdir -p "$attempt_root"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for rate in $rates; do
  tag="${rate//./p}"
  attempt_dir="$attempt_root/rate_${tag}"
  attempt_id="${run_id}_r${tag}"
  echo "FAST-LIO attempt: rate=${rate}, output=${attempt_dir}"
  if "$SCRIPT_DIR/run_mapping_from_bag.sh" \
      --bag "$bag" --run-id "$attempt_id" --runtime-root "$runtime_root" \
      --output-dir "$attempt_dir" --rate "$rate" --master-port "$port"; then
    mv "$attempt_dir" "$output_dir"
    attempt_map="$output_dir/handheld_map_${attempt_id}.pcd"
    canonical_map="$output_dir/handheld_map_${run_id}.pcd"
    if [[ -f "$attempt_map" && ! -e "$canonical_map" ]]; then
      cp --reflink=auto "$attempt_map" "$canonical_map"
    fi
    printf '%s\n' "$rate" > "$output_dir/selected_playback_rate.txt"
    echo "FAST-LIO completed at ${rate}x: $output_dir"
    exit 0
  fi
  runtime_native_map="$runtime_root/PCD/scans.pcd"
  if [[ -f "$runtime_native_map" && -d "$attempt_dir" ]]; then
    mv "$runtime_native_map" "$attempt_dir/runtime_scans_after_failed_audit.pcd"
  fi
  echo "Attempt ${rate}x did not meet coverage/trajectory-consistency checks; preserving diagnostics and trying slower playback." >&2
done

echo "All FAST-LIO playback rates failed; diagnostics remain under $attempt_root" >&2
exit 1
