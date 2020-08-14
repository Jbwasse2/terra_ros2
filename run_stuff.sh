#!/usr/bin/env bash
ROS_NODE=$1
source /opt/ros/dashing/setup.bash
source src/terra_camera/rmp_nav/set_envs.sh
. install/setup.bash
ros2 run terra_camera "$ROS_NODE"
