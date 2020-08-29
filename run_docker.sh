#!/usr/bin/env bash
ROS_NODE="$1"
DIR=/home/justin/research/ros/terra_ros2
docker run --rm \
    -v "$DIR"/src:/terra_ros2/src \
    -v "$DIR"/run_stuff.sh:/terra_ros2/run_stuff.sh \
    -v "$DIR"/requirements.txt:/terra_ros2/requirements.txt \
    -v "$DIR"/data:/terra_ros2/data \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --net=host \
    --pid=host \
    jbwasse2/ros2 \
    /terra_ros2/run_stuff.sh $ROS_NODE
    #--device /dev/video0:/dev/video0 \
    #--device /dev/video1:/dev/video1 \
