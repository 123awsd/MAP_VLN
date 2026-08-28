#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
resource_root="${PRE_MAP_VLN_RESOURCE_ROOT:-$HOME/SHILI_VLN}"
environment="${PRE_MAP_VLN_TEST_ENV:-$resource_root/.envs/pre_map_vln_tests}"
pip_cache="${PRE_MAP_VLN_PIP_CACHE:-$resource_root/cache/pip}"

mkdir -p "$resource_root/.envs" "$pip_cache"

if [[ ! -x "$environment/bin/python" ]]; then
  python3 -m venv "$environment"
fi

PIP_CACHE_DIR="$pip_cache" "$environment/bin/python" -m pip install \
  --disable-pip-version-check \
  --upgrade "pip==25.2"
PIP_CACHE_DIR="$pip_cache" "$environment/bin/python" -m pip install \
  --disable-pip-version-check \
  --requirement "$project_root/requirements-test.txt"

echo "Test environment ready: $environment"
echo "Pip cache: $pip_cache"
echo "Run: $project_root/scripts/run_unit_tests.sh"
