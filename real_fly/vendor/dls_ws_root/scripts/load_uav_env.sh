#!/usr/bin/env bash
# Source this from bash launch scripts so drone/lidar settings stored in
# /etc/uav/uav.env are visible even when bash cannot read ~/.zshrc.
#
# The file is plain bash-compatible "export VAR=value" lines, e.g.:
#   export DRONE_ID=1
#   export drone_id=1
#   export UAV_NAME=uav1
#   export LIVOX_LIDAR_TYPE=mid360
#   export LIVOX_LIDAR_IP=192.168.1.147
#
# Behaviour:
#   * The path can be overridden with UAV_ENV_FILE (default /etc/uav/uav.env).
#   * Nothing happens (no error) when the file does not exist.
#   * Variables already exported in the current environment win over the file.
#   * Only "export VAR=value" lines are applied; blank lines and comments are
#     ignored; trailing comments and surrounding quotes are stripped.
load_uav_env() {
  local env_file="${UAV_ENV_FILE:-/etc/uav/uav.env}"
  [[ -f "${env_file}" ]] || return 0
  local line key val
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line#"${line%%[![:space:]]*}"}"            # trim leading whitespace
    [[ -n "${line}" && "${line}" != \#* ]] || continue # skip blank/comment
    [[ "${line}" =~ ^(export[[:space:]]+)?([A-Za-z_][A-Za-z0-9_]*)=(.*)$ ]] || continue
    key="${BASH_REMATCH[2]}"
    val="${BASH_REMATCH[3]}"
    [[ -n "${!key:-}" ]] && continue                   # keep existing value
    val="${val%%\#*}"                                  # drop trailing comment
    val="${val%"${val##*[![:space:]]}"}"               # trim trailing whitespace
    val="${val#"${val%%[![:space:]]*}"}"               # trim leading whitespace
    if [[ "${#val}" -ge 2 ]]; then                     # strip surrounding quotes
      if [[ "${val:0:1}" == '"' && "${val: -1}" == '"' ]]; then
        val="${val:1:${#val}-2}"
      elif [[ "${val:0:1}" == "'" && "${val: -1}" == "'" ]]; then
        val="${val:1:${#val}-2}"
      fi
    fi
    export "${key}=${val}"
    case "${key^^}" in
      *TOKEN*|*PASSWORD*|*SECRET*|*KEY*)
        echo "[env] ${key}=<redacted> (from ${env_file})" >&2
        ;;
      *)
        echo "[env] ${key}=${val} (from ${env_file})" >&2
        ;;
    esac
  done < "${env_file}"
}
load_uav_env
