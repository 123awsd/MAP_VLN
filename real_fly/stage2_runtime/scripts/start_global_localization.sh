#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /absolute/path/to/approved_map.pcd" >&2
  exit 2
fi

map_pcd="$1"
dls_ws="${DLS_WS:-/home/nv/dls_ws}"
[[ "$map_pcd" = /* ]] || { echo "Map path must be absolute." >&2; exit 2; }
[[ -s "$map_pcd" ]] || { echo "Approved map is missing or empty: $map_pcd" >&2; exit 2; }
[[ -x "$dls_ws/fly.sh" ]] || { echo "Missing senior localization launcher: $dls_ws/fly.sh" >&2; exit 2; }

echo "Starting persistent global localization against: $map_pcd"
echo "This starts MAVROS, MID-360, global FAST-LIO relocalization and /ekf_quat/ekf_odom."
echo "It does not start PX4Ctrl, SUPER, arm, take off, land, or send setpoints."
echo "Keep this terminal running until the aircraft is disarmed after landing."
exec "$dls_ws/fly.sh" global "$map_pcd"
