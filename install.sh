#!/bin/bash

echo "╔══╣ Install: PCL Object Detection (STARTING) ╠══╗"


# Keep track of the current directory
CURRENT_DIR=`pwd`
cd ..

# Download required packages for SOBIT PRO
git clone https://github.com/TeamSOBITS/sobits_interfaces.git

# Go back to previous directory
cd ${CURRENT_DIR}

# Download ROS dependencies
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-laser-geometry


echo "╚══╣ Install: PCL Object Detection (FINISHED) ╠══╝"
