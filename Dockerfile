FROM ros:dashing

# install ros package
#RUN apt-get update && apt-get install -y \
#      ros-${ROS_DISTRO}-demo-nodes-cpp \
#      ros-${ROS_DISTRO}-demo-nodes-py && \
#    rm -rf /var/lib/apt/lists/*

RUN mkdir /terra_ros2
COPY terra_ros2 /terra_ros2
# launch ros package
WORKDIR "/terra_ros2"
RUN apt-get update
RUN apt-get -y install python3-pip
RUN pip3 install -U -r requirements.txt
RUN apt-get -y install ros-dashing-vision-opencv

RUN colcon build
#CMD ["bash"]
#RUN /bin/bash -c "source /opt/ros/dashing/setup.bash && . install/setup.bash && ros2 run terra_camera waypoint_publisher"
CMD ["./run_stuff.sh"]
