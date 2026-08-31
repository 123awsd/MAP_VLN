#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <stage1-run-dir> <stage1-bag>" >&2
  exit 2
fi

run_dir="$(realpath -m "$1")"
bag_path="$(realpath -m "$2")"
outputs_root="$root_dir/outputs"
export_dir="$run_dir/bag_export"

case "$run_dir/" in "$outputs_root/"*) ;; *)
  echo "stage1 run must be below $outputs_root: $run_dir" >&2; exit 2;;
esac
case "$bag_path" in "$outputs_root"/*) ;; *)
  echo "Bag must be below $outputs_root so the ROS container can read it: $bag_path" >&2; exit 2;;
esac
[[ -f "$bag_path" ]] || { echo "missing Bag: $bag_path" >&2; exit 2; }
[[ -f "$run_dir/generated_3d_config.json" ]] || {
  echo "not a Stage1 3-D run directory (missing generated_3d_config.json): $run_dir" >&2
  exit 2
}
[[ ! -e "$export_dir" ]] || {
  echo "target already exists; refusing to overwrite: $export_dir" >&2
  exit 2
}

bag_rel="${bag_path#"$outputs_root/"}"
export_rel="${export_dir#"$outputs_root/"}"
pre_map_vln_resolve_docker
cd "$root_dir"
"${docker_cmd[@]}" compose run --rm \
  -v "$root_dir/scripts/export_stage1_bag.py:/tmp/export_stage1_bag.py:ro" \
  falcon python3 /tmp/export_stage1_bag.py \
  --bag "/workspace/shared/outputs/$bag_rel" \
  --output-dir "/workspace/shared/outputs/$export_rel"

echo "Stage1 export: $export_dir"

