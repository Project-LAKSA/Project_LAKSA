#!/usr/bin/env bash
# Simulation-only entrypoint. It sources only the isolated image workspace.
set -eo pipefail
mkdir -p /tmp/ros-logs /tmp/numba /tmp/cache /tmp/home
set +u
source /opt/ros/humble/setup.bash
source /opt/laksa_ws/install/setup.bash
set -u
export ROS_LOG_DIR=/tmp/ros-logs
export NUMBA_CACHE_DIR=/tmp/numba
export XDG_CACHE_HOME=/tmp/cache
export HOME=/tmp/home
exec "$@"
