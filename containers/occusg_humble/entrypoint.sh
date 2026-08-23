#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /workspace/occusg_ws/install/setup.bash
exec "$@"
