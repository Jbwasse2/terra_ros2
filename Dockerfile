FROM ros:dashing

# install ros package
#RUN apt-get update && apt-get install -y \
#      ros-${ROS_DISTRO}-demo-nodes-cpp \
#      ros-${ROS_DISTRO}-demo-nodes-py && \
#    rm -rf /var/lib/apt/lists/*

RUN mkdir /terra_ros2
COPY terra_ros2 /terra_ros2
COPY rmp_nav /terra_ros2/rmp_nav
# launch ros package
WORKDIR "/terra_ros2"
RUN apt-get update
RUN apt-get -y install ros-dashing-vision-opencv
RUN apt-get -y install vim
RUN apt-get -y install libcairo2-dev
RUN apt-get -y install python3-pip
RUN pip3 install --upgrade pip
RUN pip3 install -r requirements.txt --ignore-installed
RUN pip3 install torch torchvision
RUN pip3 install \
    -f https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-18.04 \
    wxPython
RUN pip install Cython
RUN pip3 install Cython
RUN apt-get -y install python3-pybind11
RUN apt-get -y install python-pybind11

#Handle rmp_nav stuff
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10
ENV DIR=/terra_ros2/rmp_nav
ENV PYTHONPATH=/terra_ros2/rmp_nav
ENV RMP_NAV_ROOT=/terra_ros2/rmp_nav

RUN /terra_ros2/rmp_nav/tools/build_rangelibc.sh
RUN /terra_ros2/rmp_nav/tools/compile_python_cpp_libs.sh
RUN python -c "import numpy"


CMD ["bash"]
