#!/usr/bin/env bash
# Publish a circular PositionCommand to /planning/pos_cmd for PX4Ctrl
# controller testing. Uses the traj_server circle node, which auto-stops and
# exits after `laps` (default 3) with shutdown_after_finish.
#
# Extra roslaunch args are forwarded, e.g.:
#   ./circle.sh radius:=2.0 speed:=1.5 laps:=2
#   ./circle.sh odom_topic:=/Odometry
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

exec roslaunch traj_server circle_trajectory_cmd.launch "$@"
