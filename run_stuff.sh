#!/usr/bin/env bash
ROS_NODE=$1

colcon build
. install/setup.sh
sudo pip uninstall -y opencv-python
sudo usermod -a -G video developer
export DIR=/terra_ros2/rmp_nav
export LC_ALL="en_US.UTF-8"
export LC_CTYPE="en_US.UTF-8"
export PYTHONPATH="${DIR}":$PYTHONPATH
export RMP_NAV_ROOT="${DIR}"
ros2 run terra_camera "$ROS_NODE"
/bin/bash
