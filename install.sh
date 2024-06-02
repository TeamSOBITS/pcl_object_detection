#!/bin/bash

echo "╔══╣ Install: PCL Object Detection (STARTING) ╠══╗"


# Keep track of the current directory
CURRENT_DIR=`pwd`
cd ..

# Download required packages for SOBIT PRO
git clone https://github.com/TeamSOBITS/sobits_msgs

# Go back to previous directory
cd ${CURRENT_DIR}

# Download ROS dependencies
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-dynamic-reconfigure \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-message-generation \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-nodelet \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs


echo "╚══╣ Install: PCL Object Detection (FINISHED) ╠══╝"
