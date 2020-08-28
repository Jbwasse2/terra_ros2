#!/usr/bin/env bash

#Setup stuff
source /opt/ros/dashing/setup.zsh
source /home/justin/research/ros/terra_ros2/terra_ros2/install/setup.zsh

#################### WAYPOINT PREDICTOR#########################
#If we are using the lab computer
/home/justin/research/ros/terra_ros2/terra_ros2/run_docker_gpu.sh waypoint_predictor & 2>&1

#If we are using my personal computer
#ssh justin@192.168.1.55 '/home/justin/research/ros/terra_ros2/terra_ros2/run_docker_gpu.sh waypoint_predictor'
################################################################


################## CAMERA STREAM ###############################
#To play the bag
watch -n 1 ros2 bag play /home/justin/research/ros/terra_ros2/terra_ros2/data/bags/rosbag2_2020_08_27-10_07_41 & 2>&1

#To play the camera stream
#/home/justin/research/ros/terra_ros2/terra_ros2/run_docker.sh webcamera_publisher &

################################################################


#################### TERRA COMMS ###############################
/home/justin/research/ros/terra_ros2/terra_ros2/run_docker.sh terra_comm_twist & 2>&1
################################################################
