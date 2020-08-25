#!/usr/bin/env bash
ROS_NODE="$1"
docker run -it --rm \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/src:/terra_ros2/src \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/install:/terra_ros2/install \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/run_stuff.sh:/terra_ros2/run_stuff.sh \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/requirements.txt:/terra_ros2/requirements.txt \
    --device /dev/video0:/dev/video0 \
    --device /dev/video1:/dev/video1 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --net=host \
    --pid=host \
    jbwasse2/ros2 \
    /terra_ros2/run_stuff.sh $ROS_NODE
