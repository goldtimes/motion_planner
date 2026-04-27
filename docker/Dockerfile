# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /motion_ws

SHELL ["/bin/bash", "-c"]

# Install additional dependencies if needed
RUN apt-get update --fix-missing\
    && apt-get -y --no-install-recommends install \
    git \
    gcc \
    vim \
    psmisc \
    libxml2-dev \
    libxslt-dev \
    python3 \
    python3-pip \ 
    python-is-python3\
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-navfn \
    tmux \
    ros-noetic-gazebo-ros-pkgs

# python packages
RUN pip3 install setuptools && pip3 install catkin-tools && pip3 install conan==1.59.0 && pip3 install tmuxp

# bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Copy the project into the container
# COPY . /motion_ws

# catkin build
# RUN cd ./3rd && source ./conan_install.sh 
# && cd ../ && source /opt/ros/noetic/setup.bash && catkin_make
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'