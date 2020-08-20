#!/usr/bin/env bash
ROS_NODE="$1"
docker run -it --rm \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/src:/terra_ros2/src \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/install:/terra_ros2/install \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/run_stuff.sh:/terra_ros2/run_stuff.sh \
    -v /home/justin/research/ros/terra_ros2/terra_ros2/requirements.txt:/terra_ros2/requirements.txt \
    --network host  \
    jbwasse2/ros2 \
    /bin/sh -c "/terra_ros2/run_stuff.sh $ROS_NODE"
#   /bin/sh
