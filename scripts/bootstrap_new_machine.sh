#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/lib/docker.sh"

download_weights=false
if [[ $# -gt 1 ]]; then
  echo "usage: $0 [--download-boxer-weights]" >&2
  exit 2
elif [[ "${1:-}" == "--download-boxer-weights" ]]; then
  download_weights=true
elif [[ $# -gt 0 ]]; then
  echo "usage: $0 [--download-boxer-weights]" >&2
  exit 2
fi

"$root_dir/scripts/fetch_dependencies.sh"
"$root_dir/scripts/setup_host_envs.sh"
if [[ "$download_weights" == "true" ]]; then
  (cd "$root_dir/third_party/boxer" && ./scripts/download_ckpts.sh)
fi
pre_map_vln_resolve_docker
(cd "$root_dir" && "${docker_cmd[@]}" compose build falcon occusg)
"$root_dir/scripts/check_reproducibility.py"
