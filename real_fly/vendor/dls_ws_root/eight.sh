#!/usr/bin/env bash
# Publish a figure-8 (Lissajous) PositionCommand to /planning/pos_cmd for
# PX4Ctrl controller testing. Uses the traj_server eight node; defaults to 3
# periods then auto-exit (num_periods:=0 for an infinite loop).
#
# Extra roslaunch args are forwarded, e.g.:
#   ./eight.sh amplitude_x:=2.0 amplitude_y:=1.0 period:=10.0
#   ./eight.sh num_periods:=5
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

exec roslaunch traj_server eight_trajectory_cmd.launch "$@"
