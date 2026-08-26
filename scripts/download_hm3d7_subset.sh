#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
benchmark_root="${PRE_MAP_VLN_BENCHMARK_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2}"
selection="${PRE_MAP_VLN_HM3D_SELECTION:-$root_dir/config/hm3d7_v2.json}"
credential_file="$root_dir/.secrets/hm3d_curl.conf"
dataset_root="$benchmark_root/scenes/hm3d"
split="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["split"])' "$selection")"
mapfile -t scenes < <(python3 -c 'import json,sys; print(*json.load(open(sys.argv[1]))["scenes"],sep="\n")' "$selection")

if [[ ! -f "$credential_file" ]]; then
  echo "HM3D credentials missing. Run: .envs/habitat/bin/python scripts/configure_hm3d_credentials.py" >&2
  exit 3
fi
if (( ${#scenes[@]} != 7 )); then
  echo "selection must contain exactly 7 scenes" >&2
  exit 4
fi
mkdir -p "$dataset_root/$split" "$benchmark_root/tmp" "$benchmark_root/logs"

# Matterport authenticates the resource request and then redirects to a signed
# storage URL. Do not use --location-trusted here: forwarding the Basic auth
# header to signed storage makes S3 reject the request as double-authenticated.
# Probe the tiny semantic-config archive first so invalid credentials fail
# clearly before tar or a 27 GB stream is started.
auth_probe="$(mktemp "$benchmark_root/tmp/hm3d_auth_probe.XXXXXX")"
auth_status="$(curl --silent --show-error --location \
  --config "$credential_file" --max-time 60 --output "$auth_probe" \
  --write-out '%{http_code}' \
  "https://api.matterport.com/resources/habitat/hm3d-${split}-semantic-configs-v0.2.tar" || true)"
if [[ "$auth_status" != "200" ]] || ! tar -tf "$auth_probe" >/dev/null 2>&1; then
  rm -f "$auth_probe"
  echo "HM3D_AUTH_FAILED http_status=${auth_status:-request_error}: the official download account is invalid or lacks HM3D access" >&2
  exit 9
fi
rm -f "$auth_probe"

components=(
  "habitat|https://api.matterport.com/resources/habitat/hm3d-${split}-habitat-v0.2.tar"
  "semantic_annots|https://api.matterport.com/resources/habitat/hm3d-${split}-semantic-annots-v0.2.tar"
  "semantic_configs|https://api.matterport.com/resources/habitat/hm3d-${split}-semantic-configs-v0.2.tar"
)

for item in "${components[@]}"; do
  component="${item%%|*}"
  url="${item#*|}"
  missing=()
  if [[ "$component" == "semantic_configs" ]]; then
    [[ -f "$dataset_root/hm3d_annotated_basis.scene_dataset_config.json" ]] || missing+=("global_config")
  else
    for scene in "${scenes[@]}"; do
      case "$component" in
        habitat) pattern="$dataset_root/$split/$scene"/*.basis.glb ;;
        semantic_annots) pattern="$dataset_root/$split/$scene"/*.semantic.glb ;;
      esac
      compgen -G "$pattern" >/dev/null || missing+=("$scene")
    done
  fi
  if (( ${#missing[@]} == 0 )); then
    echo "SKIP $component: all selected files already present"
    continue
  fi
  part_dir="$(mktemp -d "$benchmark_root/tmp/hm3d_${component}.XXXXXX")"
  patterns=()
  if [[ "$component" == "semantic_configs" ]]; then
    echo "DOWNLOAD $component: extracting the global annotated scene config"
    patterns+=("hm3d_annotated_basis.scene_dataset_config.json")
  else
    echo "DOWNLOAD $component: streaming official $split archive; extracting ${#missing[@]} selected scenes"
    for scene in "${missing[@]}"; do patterns+=("$scene/*"); done
  fi
  set +e
  curl --fail --show-error --location --retry 5 --retry-delay 5 \
    --config "$credential_file" "$url" \
    | tar -x -C "$part_dir" --wildcards --no-anchored "${patterns[@]}"
  codes=("${PIPESTATUS[@]}")
  set -e
  if (( codes[0] != 0 || codes[1] != 0 )); then
    echo "download/extraction failed for $component (curl=${codes[0]} tar=${codes[1]}); partial directory preserved: $part_dir" >&2
    exit 5
  fi
  if [[ "$component" == "semantic_configs" ]]; then
    config_source="$(find "$part_dir" -name hm3d_annotated_basis.scene_dataset_config.json -print -quit)"
    [[ -n "$config_source" ]] || { echo "annotated scene config missing" >&2; exit 7; }
    cp "$config_source" "$dataset_root/hm3d_annotated_basis.scene_dataset_config.json"
  else
    for scene in "${missing[@]}"; do
      source_dir="$(find "$part_dir" -type d -name "$scene" -print -quit)"
      if [[ -z "$source_dir" ]]; then
        echo "$component archive did not contain selected scene $scene" >&2
        exit 6
      fi
      mkdir -p "$dataset_root/$split/$scene"
      cp -a "$source_dir/." "$dataset_root/$split/$scene/"
    done
  fi
  rm -rf "$part_dir"
done

python3 - "$dataset_root" "$split" "${scenes[@]}" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]); split=sys.argv[2]; scenes=sys.argv[3:]
required=("*.basis.glb","*.basis.navmesh","*.semantic.glb","*.semantic.txt")
report=[]
for scene in scenes:
    directory=root/split/scene
    missing=[pattern for pattern in required if not list(directory.glob(pattern))]
    report.append({"scene_id":scene,"directory":str(directory),"missing":missing,"passed":not missing})
payload={"format":"pre_map_vln.hm3d_subset_download.v1","scene_count":len(report),"scenes":report}
path=root.parent/"hm3d7_download_report.json"
path.write_text(json.dumps(payload,indent=2)+"\n",encoding="utf-8")
print(json.dumps(payload,indent=2))
if any(item["missing"] for item in report): raise SystemExit(8)
PY
