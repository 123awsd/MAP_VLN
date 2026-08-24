#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python="$project_root/.envs/vlnce_legacy/bin/python"
target="$project_root/checkpoints/baselines/opennav"
torchvision_target="$project_root/cache/torch/hub/checkpoints"
mkdir -p "$target"
mkdir -p "$torchvision_target"

verify_sha256() {
  local file="$1"
  local expected="$2"
  local actual
  actual="$(sha256sum "$file")"
  actual="${actual%% *}"
  if [[ "$actual" != "$expected" ]]; then
    echo "SHA-256 mismatch: $file" >&2
    return 1
  fi
}

if [[ ! -s "$target/check_val_best_avg_wayscore" ]]; then
  "$python" -m gdown 16Vk3ummmyLvpQr16TzBL-iwZNlrELOdk \
    -O "$target/check_val_best_avg_wayscore"
fi
if [[ ! -s "$target/gibson-2plus-resnet50.pth" ]]; then
  curl -L -o "$target/gibson-2plus-resnet50.pth" \
    https://zenodo.org/records/6634113/files/gibson-2plus-resnet50.pth
fi
if [[ ! -s "$torchvision_target/resnet50-0676ba61.pth" ]]; then
  curl -L --retry 5 --retry-all-errors \
    -o "$torchvision_target/resnet50-0676ba61.pth" \
    https://download.pytorch.org/models/resnet50-0676ba61.pth
fi

verify_sha256 \
  "$target/check_val_best_avg_wayscore" \
  fc2a1b92d25a9de8f947f3a6cbb125a7d559503c5c02418b252c551e7158beb1
verify_sha256 \
  "$target/gibson-2plus-resnet50.pth" \
  a6a600277efacf5fd98e293267221185d843eb3012aeff62fabfeee24c2bcdad
verify_sha256 \
  "$torchvision_target/resnet50-0676ba61.pth" \
  0676ba61b6795bbe1773cffd859882e5e297624d384b6993f7c9e683e722fb8a

"$project_root/scripts/link_external_baseline_assets.sh"
