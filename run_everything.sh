#!/usr/bin/env bash

#Setup stuff
source /opt/ros/dashing/setup.sh

#################### WAYPOINT PREDICTOR#########################
#If we are using the lab computer
2>/dev/null 1>/dev/null /home/justin/research/ros/terra_ros2/run_docker_gpu.sh waypoint_publisher &

#If we are using my personal computer
#ssh justin@192.168.1.55 '/home/justin/research/ros/terra_ros2/terra_ros2/run_docker_gpu.sh waypoint_predictor'
################################################################


################## CAMERA STREAM ###############################
#To play the bag
2>/dev/null 1>/dev/null ./run_bag.sh &

#To play the camera stream
#/home/justin/research/ros/terra_ros2/terra_ros2/run_docker.sh webcamera_publisher &

################################################################


#################### TERRA COMMS ###############################
#2>/dev/null 1>/dev/null home/justin/research/ros/terra_ros2/run_docker.sh terra_comm_twist &
################################################################
