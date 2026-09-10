#!/usr/bin/env bash
# Export the small, approved planning inputs for one RUN_ID to the NX runtime.
# This never starts ROS, hardware, a planner/controller, or any flight process.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  export_real_planning_package.sh --run-id ID [--sync-nx]

Options:
  --sync-nx          Also copy the immutable package to the configured NX.
  --nx-host HOST     SSH destination (default: nv@192.168.0.250).
  --nx-root PATH     NX project root.
  -h, --help         Show this help.

The command refuses to overwrite an existing local or remote package.
EOF
}

run_id=""
sync_nx=0
nx_host="nv@192.168.0.250"
nx_root="/home/nv/SL_WS/PRE_MAP_VLN_real_fly"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; run_id="$2"; shift 2 ;;
    --sync-nx) sync_nx=1; shift ;;
    --nx-host) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; nx_host="$2"; shift 2 ;;
    --nx-root) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; nx_root="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || {
  echo "Invalid --run-id: $run_id" >&2
  exit 2
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stage2_dir="$(cd -- "$script_dir/.." && pwd)"
root="$(cd -- "$stage2_dir/../.." && pwd)"
run_data="$stage2_dir/data/$run_id"
runtime_root="$root/real_fly/stage2_runtime"
package="$runtime_root/map_packages/$run_id"
scene="$run_data/approved_scene_graph.json"
voxel="$run_data/voxel_snapshot"
planning_start="$run_data/planning_start.json"
config="$stage2_dir/config/uav_3d_planning_real.yaml"
map="$run_data/fastlio_complete/handheld_map_${run_id}_complete.pcd"

for path in "$scene" "$voxel/metadata.json" "$voxel/voxel_map.npz" \
  "$planning_start" "$config" "$map"; do
  [[ -s "$path" ]] || { echo "Missing planning-package input: $path" >&2; exit 2; }
done
[[ ! -e "$package" ]] || {
  echo "Refusing to overwrite existing local package: $package" >&2
  exit 2
}

mkdir -p "$package/voxel_snapshot"
cp "$scene" "$package/approved_scene_graph.json"
cp "$voxel/metadata.json" "$package/voxel_snapshot/metadata.json"
cp "$voxel/voxel_map.npz" "$package/voxel_snapshot/voxel_map.npz"
cp "$planning_start" "$package/planning_start.json"
cp "$config" "$package/uav_3d_planning_real.yaml"

python3 - "$package/manifest.json" "$run_id" "$map" "$package" <<'PY'
import hashlib
import json
import pathlib
import sys


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


target = pathlib.Path(sys.argv[1])
run_id = sys.argv[2]
map_path = pathlib.Path(sys.argv[3])
package = pathlib.Path(sys.argv[4])
files = [
    "approved_scene_graph.json",
    "planning_start.json",
    "uav_3d_planning_real.yaml",
    "voxel_snapshot/metadata.json",
    "voxel_snapshot/voxel_map.npz",
]
document = {
    "format": "pre_map_vln.nx_planning_package.v1",
    "run_id": run_id,
    "frame_id": "world",
    "map_file_name": f"{run_id}.pcd",
    "map_sha256": sha256(map_path),
    "files": {name: sha256(package / name) for name in files},
    "safety_scope": "approved offline map inputs; no flight authorization",
}
target.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

echo "Planning package ready: $package"
if [[ "$sync_nx" -eq 0 ]]; then
  exit 0
fi

remote_map="/home/nv/dls_ws/${run_id}.pcd"
expected_sha="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["map_sha256"])' "$package/manifest.json")"
actual_sha="$(ssh -o BatchMode=yes "$nx_host" \
  "test -s '$remote_map' && sha256sum '$remote_map' | cut -d' ' -f1")" || {
  echo "NX map is missing: $remote_map" >&2
  exit 2
}
[[ "$actual_sha" == "$expected_sha" ]] || {
  echo "NX map SHA256 mismatch; refusing to mix RUN_ID data." >&2
  echo "expected=$expected_sha" >&2
  echo "actual=$actual_sha" >&2
  exit 2
}

remote_parent="$nx_root/real_fly/stage2_runtime/map_packages"
remote_package="$remote_parent/$run_id"
ssh -o BatchMode=yes "$nx_host" \
  "test ! -e '$remote_package' && mkdir -p '$remote_parent'" || {
  echo "Refusing to overwrite existing NX package: $remote_package" >&2
  exit 2
}
rsync -avP --partial "$package/" "$nx_host:$remote_package/"
echo "Planning package synced to: $nx_host:$remote_package"
