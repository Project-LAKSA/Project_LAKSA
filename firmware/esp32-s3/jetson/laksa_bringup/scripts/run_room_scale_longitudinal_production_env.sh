#!/usr/bin/env bash

# Isolated testing launcher; does not alter production runtime configuration.
set +u
source /opt/ros/humble/local_setup.bash
source /home/ubuntu/laksa_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/etc/laksa/cyclonedds.xml

exec ros2 run laksa_bringup room_scale_longitudinal_recorder.py "$@"
