#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
baseline="${1:?usage: run_vlnce_legacy.sh open-nav|vln-zero PYTHON_ARGS...}"
shift

export HF_HOME="$project_root/cache/huggingface"
export TORCH_HOME="$project_root/cache/torch"
export MPLCONFIGDIR="$project_root/cache/matplotlib"

if [[ -z "${OPENAI_API_KEY:-}" && -s "$project_root/.secrets/dashscope_api_key" ]]; then
  OPENAI_API_KEY="$(<"$project_root/.secrets/dashscope_api_key")"
  export OPENAI_API_KEY
fi
export OPENAI_BASE_URL="${OPENAI_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"
export VLN_ZERO_MODEL="${VLN_ZERO_MODEL:-qwen3.7-plus}"
export PRE_MAP_VLN_API_LEDGER="${PRE_MAP_VLN_API_LEDGER:-$project_root/outputs/baselines/qwen_usage.jsonl}"
export CUDA_VISIBLE_DEVICES="${PRE_MAP_VLN_LEGACY_CUDA_VISIBLE_DEVICES:-0}"
export PRE_MAP_VLN_FORCE_TORCH_CPU="${PRE_MAP_VLN_FORCE_TORCH_CPU:-1}"
export PRE_MAP_VLN_OPENNAV_RETRIES="${PRE_MAP_VLN_OPENNAV_RETRIES:-1}"
export PRE_MAP_VLN_OPENNAV_SAMPLES="${PRE_MAP_VLN_OPENNAV_SAMPLES:-1}"

case "$baseline" in
  open-nav)
    export PYTHONPATH="$project_root:$project_root/third_party/VLN-Zero/habitat-lab:$project_root/third_party/Open-Nav"
    export PRE_MAP_VLN_SHARED_PERCEPTION="${PRE_MAP_VLN_SHARED_PERCEPTION:-1}"
    export PRE_MAP_VLN_OWLV2_CUDA_VISIBLE_DEVICES="${PRE_MAP_VLN_OWLV2_CUDA_VISIBLE_DEVICES:-0}"
    ;;
  vln-zero)
    export PYTHONPATH="$project_root:$project_root/third_party/VLN-Zero/habitat-lab:$project_root/third_party/VLN-Zero"
    ;;
  *)
    echo "unknown baseline: $baseline" >&2
    exit 2
    ;;
esac

exec "$project_root/.envs/vlnce_legacy/bin/python" "$@"
