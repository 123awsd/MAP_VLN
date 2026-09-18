#!/usr/bin/env bash
# Start only the real-aircraft mission/planning chain in a dedicated terminal.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: ./plan.sh [full_smooth|super|hybrid] [roslaunch arguments...]

Defaults to the already validated full_smooth mission. Examples:
  ./plan.sh
  ./plan.sh full_smooth rviz:=true
  ./plan.sh hybrid rviz:=true
EOF
}

MISSION_MODE="full_smooth"
if [[ $# -gt 0 ]]; then
    case "$1" in
        full_smooth|super|hybrid)
            MISSION_MODE="$1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
    esac
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

# The launch defaults to the real competition config and locates the latest
# recorder-validated trajectory itself.  Extra roslaunch arguments are kept
# available for RViz and explicit diagnostic overrides.
exec roslaunch mission_planner competition_2026_planner.launch \
    mission_mode:="${MISSION_MODE}" "$@"
