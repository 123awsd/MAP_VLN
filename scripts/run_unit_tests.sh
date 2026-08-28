#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
resource_root="${PRE_MAP_VLN_RESOURCE_ROOT:-$HOME/SHILI_VLN}"
environment="${PRE_MAP_VLN_TEST_ENV:-$resource_root/.envs/pre_map_vln_tests}"
python="$environment/bin/python"

if [[ ! -x "$python" ]]; then
  echo "Test environment is missing: $environment" >&2
  echo "Create it with: $project_root/scripts/setup_test_env.sh" >&2
  exit 2
fi

cd "$project_root"
"$python" -m unittest discover -s tests -v
