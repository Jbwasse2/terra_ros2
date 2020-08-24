#!/usr/bin/env bash
ROS_NODE="$1"
docker run -it --rm \
    -v /home/companion/research/ros/terra_ros2/terra_ros2/src:/terra_ros2/src \
    -v /home/companion/research/ros/terra_ros2/terra_ros2/install:/terra_ros2/install \
    -v /home/companion/research/ros/terra_ros2/terra_ros2/run_stuff.sh:/terra_ros2/run_stuff.sh \
    -v /home/companion/research/ros/terra_ros2/terra_ros2/requirements.txt:/terra_ros2/requirements.txt \
    --privileged \
    --device /dev/video0:/dev/video0 \
    --device /dev/video1:/dev/video1 \
    --device /dev/video2:/dev/video2 \
    --device /dev/video3:/dev/video3 \
    -p 5000:5000 \
    -p 8888:8888 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --net=host \
    --pid=host \
    --user developer \
    jbwasse2/ros2 \
    /terra_ros2/run_stuff.sh $ROS_NODE
