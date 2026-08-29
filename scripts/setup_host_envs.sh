#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
conda_bin="${CONDA_EXE:-$(command -v conda || true)}"
torch_index="${PRE_MAP_VLN_TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu130}"
torch_version="${PRE_MAP_VLN_TORCH_VERSION:-2.13.0}"

if [[ -z "$conda_bin" ]]; then
  echo "conda was not found. Install Miniconda/Miniforge first." >&2
  exit 2
fi
if [[ ! -f "$root_dir/third_party/habitat-lab/habitat-lab/setup.py" ]]; then
  echo "third-party sources are missing; run ./scripts/fetch_dependencies.sh first." >&2
  exit 2
fi

if [[ ! -x "$root_dir/.envs/habitat/bin/python" ]]; then
  "$conda_bin" create -y --prefix "$root_dir/.envs/habitat" \
    -c conda-forge -c aihabitat \
    python=3.9 habitat-sim=0.3.3 withbullet cmake=3.27.9
fi
"$root_dir/.envs/habitat/bin/python" -m pip install \
  -e "$root_dir/third_party/habitat-lab/habitat-lab"
"$root_dir/.envs/habitat/bin/python" -m pip install \
  -r "$root_dir/requirements/habitat-pip.txt"

if [[ ! -x "$root_dir/.envs/boxer/bin/python" ]]; then
  "$conda_bin" create -y --prefix "$root_dir/.envs/boxer" -c conda-forge python=3.12
fi
"$root_dir/.envs/boxer/bin/python" -m pip install \
  "torch==$torch_version" --index-url "$torch_index"
"$root_dir/.envs/boxer/bin/python" -m pip install \
  -r "$root_dir/requirements/boxer-pip.txt"

echo "Host environments are ready. Run ./scripts/check_reproducibility.py next."
