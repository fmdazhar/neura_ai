FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO noetic
ARG DEBIAN_FRONTEND=noninteractive

# Basic setup
RUN apt-get update && apt-get install --no-install-recommends -y \
    locales \
    lsb-release \
    curl \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    build-essential \
    wget \
    nano \
    python3-pip \
    python3-catkin-tools \
    tmux \
    pkg-config \
    xorg-dev libxinerama-dev libxcursor-dev libglu1-mesa-dev mesa-utils \
    libgtk2.0-dev \
    ros-noetic-moveit && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install numpy \
    matplotlib \
    pandas \
    scipy \
    scikit-learn \
    scikit-image \
    opencv-python \
    opencv-python-headless \
    pyqt5 \
    pyqtgraph

# Set locale
RUN dpkg-reconfigure locales

# Initialize rosdep
RUN rosdep fix-permissions && rosdep update --rosdistro $ROS_DISTRO

# source ROS setup.bash on startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Copy directories
COPY ./catkin_ws /root/catkin_ws

# Setup workspace
WORKDIR /root/catkin_ws

RUN source /opt/ros/noetic/setup.bash && \
    apt-get update && \
    rosdep install -q -y --from-paths ./src --ignore-src --rosdistro noetic && \
    rm -rf /var/lib/apt/lists/*

# Build workspace
RUN source /opt/ros/noetic/setup.bash && catkin build

# Source workspace setup files on startup
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Add aliases
RUN echo "alias source_ws='source /root/catkin_ws/devel/setup.bash'" >> ~/.bashrc
