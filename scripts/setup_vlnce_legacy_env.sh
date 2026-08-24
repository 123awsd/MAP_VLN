#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
environment="$project_root/.envs/vlnce_legacy"
conda_executable="${PRE_MAP_VLN_CONDA:-/home/uav/miniconda3/bin/conda}"
package_cache="$project_root/cache/conda-pkgs"
pip_cache="$project_root/cache/pip"

if [[ ! -x "$environment/bin/python" ]]; then
  CONDA_PKGS_DIRS="$package_cache" "$conda_executable" create -y \
    -p "$environment" -c conda-forge -c aihabitat \
    python=3.8.19 pip=24.2 habitat-sim=0.1.7 headless
fi

git -C "$project_root/third_party/VLN-Zero" submodule update --init --depth 1 habitat-lab
"$project_root/scripts/apply_external_baseline_patches.sh"

PIP_CACHE_DIR="$pip_cache" "$environment/bin/python" -m pip install \
  -e "$project_root/third_party/VLN-Zero/habitat-lab"
PIP_CACHE_DIR="$pip_cache" "$environment/bin/python" -m pip install \
  torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cu121
PIP_CACHE_DIR="$pip_cache" "$environment/bin/python" -m pip install \
  opencv-python==4.10.0.84 gym==0.17.3 dtw==1.4.0 fastdtw==0.3.4 \
  gdown jsonlines msgpack-numpy lmdb==1.7.3 moviepy==1.0.3 \
  webdataset==0.1.40 openai python-dotenv absl-py boto3 ifcfg==0.24 tenacity==9.0.0 \
  tensorboard==2.14.0 pytorch-transformers==1.2.0 \
  transformers==4.44.0 accelerate==0.33.0
PIP_CACHE_DIR="$pip_cache" "$environment/bin/python" -m pip install \
  numpy==1.23.5

echo "Legacy VLN-CE environment ready at $environment"
echo "Note: upstream torch 2.4.1 does not provide RTX 5060 Ti sm_120 kernels."
