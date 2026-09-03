#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE1_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/env.sh"

runtime_root="${STAGE1_ROOT}/runtime/fast_lio"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime-root) [[ $# -ge 2 ]] || { echo "--runtime-root requires a path." >&2; exit 2; }; runtime_root="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--runtime-root /absolute/path/under/stage1]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$runtime_root" in
  "$STAGE1_ROOT"/*) ;;
  *) echo "Runtime root must be under $STAGE1_ROOT; no external output is allowed." >&2; exit 2 ;;
esac
if [[ -e "$runtime_root" ]]; then
  leftovers="$(/usr/bin/find "$runtime_root" -mindepth 1 -maxdepth 1 ! -name Log ! -name PCD -print -quit 2>/dev/null)"
  [[ -z "$leftovers" ]] || { echo "Refusing to build into a non-empty runtime root: $runtime_root" >&2; exit 2; }
  for subdir in Log PCD; do
    [[ ! -e "$runtime_root/$subdir" || -z "$(/usr/bin/find "$runtime_root/$subdir" -mindepth 1 -print -quit 2>/dev/null)" ]] || {
      echo "Refusing to build into a non-empty runtime root: $runtime_root/$subdir" >&2
      exit 2
    }
  done
fi
command -v catkin_make >/dev/null 2>&1 || { echo "catkin_make not found." >&2; exit 1; }
[[ -f "$STAGE1_ROOT/ros_ws/src/stage1_fast_lio/CMakeLists.txt" ]] || { echo "Stage-1 package missing." >&2; exit 1; }
mkdir -p "$runtime_root/Log" "$runtime_root/PCD"
cd "$STAGE1_ROOT/ros_ws"
catkin_make -DCMAKE_BUILD_TYPE=Release -DSTAGE1_LIO_ROOT="$runtime_root"
echo "Built Stage-1 FAST-LIO overlay under $STAGE1_ROOT/ros_ws/devel"
echo "FAST-LIO runtime root: $runtime_root"
