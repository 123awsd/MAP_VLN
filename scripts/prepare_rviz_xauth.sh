#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
display="${DISPLAY:-:0}"
target_auth="$root_dir/runtime/rviz.Xauthority"

source_auth="${XAUTHORITY:-}"
if [[ -z "$source_auth" || ! -r "$source_auth" ]]; then
  source_auth="/run/user/$(id -u)/gdm/Xauthority"
fi
if [[ ! -r "$source_auth" && -r "$HOME/.Xauthority" ]]; then
  source_auth="$HOME/.Xauthority"
fi
if [[ ! -r "$source_auth" ]]; then
  echo "找不到可读的 X11 授权文件，无法把 RViz 显示到当前桌面。" >&2
  exit 1
fi

mkdir -p "$root_dir/runtime"
cookie="$(xauth -i -f "$source_auth" nlist "$display" | sed -e 's/^..../ffff/')"
if [[ -z "$cookie" ]]; then
  echo "X11 授权文件中没有 DISPLAY=$display 的 cookie。" >&2
  exit 1
fi

xauth -f "$target_auth" remove "$display" >/dev/null 2>&1 || true
printf '%s\n' "$cookie" | xauth -f "$target_auth" nmerge -
chmod 600 "$target_auth"
