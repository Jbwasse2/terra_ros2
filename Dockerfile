FROM osrf/ros:dashing-desktop
# install ros package
#RUN apt-get update && apt-get install -y \
#      ros-${ROS_DISTRO}-demo-nodes-cpp \
#      ros-${ROS_DISTRO}-demo-nodes-py && \
#    rm -rf /var/lib/apt/lists/*

RUN mkdir /terra_ros2
COPY terra_ros2 /terra_ros2
COPY rmp_nav /terra_ros2/rmp_nav
#rmp for network stuff
ENV RTI_NC_LICENSE_ACCEPTED yes

# bootstrap rosdep
RUN rosdep update

# install dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && apt-get update \
    && rosdep install -y \
    --from-paths /opt/ros/$ROS_DISTRO/share \
    --ignore-src \
    --skip-keys " \
      " \
    && rm -rf /var/lib/apt/lists/*


# set up environment
ENV NDDSHOME /opt/rti.com/rti_connext_dds-5.3.1
ENV PATH "$NDDSHOME/bin":$PATH
ENV LD_LIBRARY_PATH "$NDDSHOME/lib/x64Linux3gcc5.4.0":$LD_LIBRARY_PATH
# bootstrap rosdep
RUN rosdep update --rosdistro $ROS_DISTRO

# install dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && apt-get update \
    && rosdep install -y \
    --from-paths /opt/ros/$ROS_DISTRO/share \
    --ignore-src \
    --skip-keys " \
      " \
    && rm -rf /var/lib/apt/lists/*


# set up environment
ENV NDDSHOME /opt/rti.com/rti_connext_dds-5.3.1
ENV PATH "$NDDSHOME/bin":$PATH
ENV LD_LIBRARY_PATH "$NDDSHOME/lib/x64Linux3gcc5.4.0":$LD_LIBRARY_PATH
# launch ros package
WORKDIR "/terra_ros2"
RUN apt-get update
RUN apt-get -y install ros-dashing-vision-opencv
RUN apt-get -y install vim
RUN apt-get -y install libcairo2-dev
RUN apt-get -y install python3-pip
RUN pip3 install --upgrade pip
RUN pip3 install -r requirements.txt --ignore-installed
RUN pip3 install torch==1.3.1 torchvision==0.4.2+cu92 -f https://download.pytorch.org/whl/torch_stable.html
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

ENV PATH /usr/local/cuda/bin/:$PATH
ENV LD_LIBRARY_PATH /usr/local/cuda/lib:/usr/local/cuda/lib64
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

LABEL com.nvidia.volumes.needed="nvidia_driver"

RUN /terra_ros2/rmp_nav/tools/build_rangelibc.sh
RUN /terra_ros2/rmp_nav/tools/compile_python_cpp_libs.sh

#Install graphics stuff to comm with X11
# Install vnc, xvfb in order to create a 'fake' display and firefox
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

USER developer
RUN sudo usermod -a -G video developer
ENV HOME /home/developer

CMD ["bash"]
