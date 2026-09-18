#!/usr/bin/env bash
# Start only the real-aircraft PX4Ctrl process in a dedicated terminal.
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"
set -u

# run_ctrl.launch defaults to ctrl_param_fpv.yaml.  Forwarding
# launch arguments preserves one-off diagnostics such as odom_topic:=... .
exec roslaunch px4ctrl run_ctrl.launch "$@"
