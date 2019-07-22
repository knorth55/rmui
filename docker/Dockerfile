ARG ROS_DISTRO
FROM knorth55/raspbian_ros:${ROS_DISTRO}-latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

RUN mkdir ~/ros/rmui_ws/src -p && \
    cd ~/ros/rmui_ws/src && \
    wget https://raw.githubusercontent.com/knorth55/rmui/master/kinetic.rosinstall?token=ACG6QX5SPCICFKYXZK6VR725H2OKI -O .rosinstall && \
    wstool update -j 2&& \
    rosdep install --ignore-src --from-path . -y -r -c

ARG ROS_DISTRO
RUN source ~/ros/${ROS_DISTRO}/devel/setup.bash && \
    cd ~/ros/rmui_ws && \
    catkin config && \
    catkin build -j 4

RUN rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
