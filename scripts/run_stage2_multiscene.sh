#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"
exec .envs/habitat/bin/python scripts/benchmark_stage2_maps.py "$@"
