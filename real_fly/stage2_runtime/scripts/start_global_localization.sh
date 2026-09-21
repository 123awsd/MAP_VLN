#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<EOF
Usage:
  $0 /absolute/path/to/approved_map.pcd
  $0 /absolute/path/to/approved_map.pcd --scdb /absolute/path/to/map.scdb

Localization always uses the validated automatic Scan Context database path.
No command-line mode directly assigns the current pose.
EOF
  exit 2
}

[[ $# -ge 1 ]] || usage
map_pcd="$1"
shift
scdb_path=""
case "${1:-}" in
  "") ;;
  --scdb)
    [[ $# -eq 2 ]] || usage
    scdb_path="$2"
    [[ "$scdb_path" = /* && -s "$scdb_path" ]] || {
      echo "Missing absolute Scan Context database: $scdb_path" >&2
      exit 2
    }
    ;;
  *) usage ;;
esac

dls_ws="${DLS_WS:-/home/nv/dls_ws}"
[[ "$map_pcd" = /* ]] || { echo "Map path must be absolute." >&2; exit 2; }
[[ -s "$map_pcd" ]] || { echo "Approved map is missing or empty: $map_pcd" >&2; exit 2; }
[[ -x "$dls_ws/fly.sh" ]] || { echo "Missing senior localization launcher: $dls_ws/fly.sh" >&2; exit 2; }

echo "Starting persistent global localization against: $map_pcd"
echo "This starts MAVROS, MID-360, global FAST-LIO relocalization and /ekf_quat/ekf_odom."
echo "It does not start PX4Ctrl, SUPER, arm, take off, land, or send setpoints."
echo "Keep this terminal running until the aircraft is disarmed after landing."
if [[ -n "$scdb_path" ]]; then
  echo "Initialization: full-map auto mode with explicit SCDB: $scdb_path"
  exec env INIT_MODE=auto GCS_SCDB="$scdb_path" "$dls_ws/fly.sh" global "$map_pcd"
fi
echo "Initialization: full-map auto mode; ${map_pcd}.scdb is required."
exec env INIT_MODE=auto "$dls_ws/fly.sh" global "$map_pcd"
