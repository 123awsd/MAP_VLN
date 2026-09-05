#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REAL_FLY_ROOT="$(cd "$CALIB_ROOT/.." && pwd)"
STAGE1_ROOT="$REAL_FLY_ROOT/stage1_exploration"
UPSTREAM="$CALIB_ROOT/third_party/FAST-Calib"
PATCH_FILE="$CALIB_ROOT/patches/fast-calib-livox-driver2.patch"
ROS_WS="$CALIB_ROOT/ros_ws"
UPSTREAM_URL=https://github.com/hku-mars/FAST-Calib.git
UPSTREAM_COMMIT=1018ecfdf9deda51b91a8a11bd11972a0b159008

if [[ ! -d "$UPSTREAM/.git" ]]; then
  mkdir -p "$CALIB_ROOT/third_party"
  git clone "$UPSTREAM_URL" "$UPSTREAM"
  git -C "$UPSTREAM" checkout "$UPSTREAM_COMMIT"
fi

actual_commit="$(git -C "$UPSTREAM" rev-parse HEAD)"
[[ "$actual_commit" == "$UPSTREAM_COMMIT" ]] || {
  echo "Unexpected FAST-Calib commit: $actual_commit" >&2
  echo "Expected: $UPSTREAM_COMMIT" >&2
  exit 1
}

if git -C "$UPSTREAM" apply --check "$PATCH_FILE" >/dev/null 2>&1; then
  git -C "$UPSTREAM" apply "$PATCH_FILE"
elif git -C "$UPSTREAM" apply --reverse --check "$PATCH_FILE" >/dev/null 2>&1; then
  echo "Livox driver2 compatibility patch is already applied."
else
  echo "FAST-Calib tree has unexpected changes; refusing to force the patch." >&2
  git -C "$UPSTREAM" status --short >&2
  exit 1
fi

mkdir -p "$ROS_WS/src"
package_link="$ROS_WS/src/fast_calib"
if [[ -L "$package_link" ]]; then
  [[ "$(readlink -f "$package_link")" == "$UPSTREAM" ]] || {
    echo "Unexpected symlink target: $package_link" >&2
    exit 1
  }
elif [[ -e "$package_link" ]]; then
  echo "Refusing to replace existing path: $package_link" >&2
  exit 1
else
  ln -s ../../third_party/FAST-Calib "$package_link"
fi

unset _CATKIN_SETUP_DIR || true
source /opt/ros/noetic/setup.bash
source "$STAGE1_ROOT/scripts/env.sh"

cd "$ROS_WS"
catkin_make -DCMAKE_BUILD_TYPE=Release

[[ -x "$ROS_WS/devel/lib/fast_calib/fast_calib" ]] || {
  echo "FAST-Calib binary was not produced." >&2
  exit 1
}
[[ -x "$ROS_WS/devel/lib/fast_calib/multi_fast_calib" ]] || {
  echo "FAST-Calib multi-scene binary was not produced." >&2
  exit 1
}

echo "FAST-Calib ready at commit: $UPSTREAM_COMMIT"
echo "Workspace: $ROS_WS"
