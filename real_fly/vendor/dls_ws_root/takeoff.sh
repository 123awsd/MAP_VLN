#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand \
  "takeoff_land_cmd: 1"
