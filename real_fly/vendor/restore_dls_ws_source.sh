#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [DLS_WS]" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_root="$script_dir/dls_ws_src"
dls_ws="${1:-$HOME/dls_ws}"

[[ -d "$source_root" ]] || {
  echo "Missing vendored DLS source: $source_root" >&2
  exit 1
}

mkdir -p "$dls_ws/src"
rsync -a "$source_root/" "$dls_ws/src/"

echo "Vendored real-flight DLS sources restored under: $dls_ws/src"
echo "Next: source ROS, install system dependencies, then run a Release catkin build."
