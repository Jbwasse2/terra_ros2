#!/usr/bin/env bash
ROS_NODE=$1
colcon build
. install/setup.sh
DIR=/terra_ros2/rmp_nav
export PYTHONPATH="${DIR}":$PYTHONPATH
export RMP_NAV_ROOT="${DIR}"
printenv
ros2 run terra_camera "$ROS_NODE"
