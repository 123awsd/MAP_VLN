#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

apply_once() {
  local repository="$1"
  local patch_file="$2"
  if git -C "$repository" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
    echo "already applied: $patch_file"
    return
  fi
  git -C "$repository" apply --check "$patch_file"
  git -C "$repository" apply "$patch_file"
  echo "applied: $patch_file"
}

apply_once "$project_root/third_party/Open-Nav" \
  "$project_root/patches/Open-Nav-qwen-owlv2-port.patch"
apply_once "$project_root/third_party/VLN-Zero" \
  "$project_root/patches/VLN-Zero-qwen-port.patch"
