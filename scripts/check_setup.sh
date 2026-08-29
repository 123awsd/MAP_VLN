#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fail=0
check() { if "$@" >/dev/null 2>&1; then echo "PASS $*"; else echo "FAIL $*"; fail=1; fi; }

check git --version
check docker compose version
check "$root_dir/.envs/habitat/bin/python" -c 'import habitat,habitat_sim,numpy; assert habitat_sim.__version__ == "0.3.3"; assert numpy.__version__ == "1.26.4"'
check "$root_dir/.envs/boxer/bin/python" -c 'import torch,numpy; assert torch.__version__.startswith("2.13.0"); assert int(numpy.__version__.split(".")[0]) == 2'
check docker image inspect pre-map-vln/falcon-noetic:local
check docker image inspect pre-map-vln/occusg-humble:local

if [[ $fail -ne 0 ]]; then
  echo "Setup check failed; see README prerequisites." >&2
  exit 1
fi
echo "Setup check passed."
