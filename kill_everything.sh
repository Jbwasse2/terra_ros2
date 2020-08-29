#!/usr/bin/env bash

#Get id for rosbag script
ROSBAG_ID=$(ps -aux | grep bash | grep run_bag | cut -d ' ' -f5)
kill -9 "$ROSBAG_ID"

#Get docker container ID and kill
DOCKER_ID=$(docker ps -a | grep jbwasse2/ros2 | cut -d ' ' -f1)
docker kill "$DOCKER_ID"

pkill -9 ros2
pkill -9 docker
