#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
third_party_dir="$project_root/third_party"
mkdir -p "$third_party_dir"

clone_at_commit() {
  local name="$1"
  local url="$2"
  local commit="$3"
  local recursive="${4:-false}"

  if [[ ! -d "$third_party_dir/$name/.git" ]]; then
    if [[ "$recursive" == "true" ]]; then
      git clone --recursive "$url" "$third_party_dir/$name"
    else
      git clone "$url" "$third_party_dir/$name"
    fi
  fi

  git -C "$third_party_dir/$name" fetch origin "$commit"
  git -C "$third_party_dir/$name" checkout --detach "$commit"
  if [[ "$recursive" == "true" ]]; then
    git -C "$third_party_dir/$name" submodule update --init --recursive
  fi
}

clone_at_commit FALCON https://github.com/HKUST-Aerial-Robotics/FALCON.git 312eb4d32c6c7af1a482f94a0a204aa2bb150cca
clone_at_commit boxer https://github.com/facebookresearch/boxer.git 1f86542dc342a4b1d474c87c97c5d1d6566d9148
clone_at_commit OccuSG https://github.com/crcz25/OccuSG.git 2bca2fa06af87fd9dd0542957039be8f580f0dca true
clone_at_commit habitat-lab https://github.com/facebookresearch/habitat-lab.git cdbb4880519505adf45fba0f0c0c3a3fd18a2a55
clone_at_commit Open3D https://github.com/isl-org/Open3D.git 0f06a149c4fb9406fd3e432a5cb0c024f38e2f0e
clone_at_commit nlopt https://github.com/stevengj/nlopt.git 09b3c2a6da71cabcb98d2c8facc6b83d2321ed71
clone_at_commit Open-Nav https://github.com/YanyuanQiao/Open-Nav.git 3a8dcefe5bfdab5192c3c3bf80b14fb096cb08c7
clone_at_commit VLN-Zero https://github.com/VLN-Zero/vln-zero.github.io.git 64b76cf9bbf4286803adb17477e5d0c222e3d63b true
clone_at_commit Spatial-X https://github.com/IMNearth/Spatial-X.git 9afdacd294e54c76795c25005a0c4498f85f7ddf
