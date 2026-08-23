#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /workspace/falcon_ws/devel/setup.bash
exec "$@"
