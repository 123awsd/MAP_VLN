#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_benchmark_v1}"
output_dir="$benchmark_root/reports/roofless_maps"
scenes=("$@")
if (( ${#scenes[@]} == 0 )); then
  scenes=(
    Z6MFQCViBuw zsNo4HB9uLZ 2azQ1b91cZZ QUCTc6BB5sX EU6Fwq7SyZv
    TbHJrupSAjP X7HyMhZNoso oLBMNvg9in8 x8F5xyUWy9e 8194nk5LbLH
  )
fi

mkdir -p "$output_dir"
cd "$root_dir"
for scene_id in "${scenes[@]}"; do
  bag="$benchmark_root/bags/${scene_id}_stage1_final.bag"
  source_kind="current"
  if [[ ! -f "$bag" ]]; then
    # A quality retry archives the previous, fully completed result before it
    # starts. If that retry is stopped, preserve visual coverage by exporting
    # the newest recoverable final bag rather than a partial raw bag.
    bag="$(find "$benchmark_root/recovery_snapshots/$scene_id" \
      -path "*/bags/${scene_id}_stage1_final.bag" -type f -print 2>/dev/null \
      | sort | tail -n 1)"
    source_kind="recovery_snapshot"
  fi
  if [[ -z "$bag" || ! -f "$bag" ]]; then
    echo "SKIP $scene_id: no completed final bag exists" >&2
    continue
  fi
  relative_bag="${bag#"$benchmark_root"/}"
  echo "EXPORT $scene_id source=$source_kind path=$relative_bag"
  env PRE_MAP_VLN_BENCHMARK_ROOT="$benchmark_root" docker compose run --rm falcon \
    python3 /workspace/falcon_ws/src/pre_map_bridge/scripts/export_roofless_cloud_image.py \
    --scene-id "$scene_id" \
    --bag "/workspace/benchmark/$relative_bag" \
    --output "/workspace/benchmark/reports/roofless_maps/${scene_id}_roofless.png" \
    --metadata "/workspace/benchmark/reports/roofless_maps/${scene_id}_roofless.json"
done

python3 - "$output_dir" "${scenes[@]}" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1])
items=[]
for scene in sys.argv[2:]:
    path=root/f"{scene}_roofless.json"
    if path.exists():
        items.append(json.loads(path.read_text(encoding="utf-8")))
manifest={"format":"pre_map_vln.roofless_cloud_gallery.v1","image_count":len(items),"images":items}
(root/"manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps({"output":str(root),"image_count":len(items)},ensure_ascii=False))
PY
