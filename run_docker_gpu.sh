#!/usr/bin/env bash
ROS_NODE="$1"
DIR=/home/justin/research/ros/terra_ros2
docker run --rm \
    -v "$DIR"/src:/terra_ros2/src \
    -v "$DIR"/run_stuff.sh:/terra_ros2/run_stuff.sh \
    -v "$DIR"/requirements.txt:/terra_ros2/requirements.txt \
    -v "$DIR"/data:/terra_ros2/data \
    -v /usr/local/cuda-9.1:/usr/local/cuda-9.1 \
    --net=host \
    --pid=host \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    jbwasse2/ros2 \
    /terra_ros2/run_stuff.sh $ROS_NODE
