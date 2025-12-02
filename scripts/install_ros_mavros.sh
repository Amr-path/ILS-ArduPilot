#!/bin/bash
# Install ROS Noetic and MAVROS

set -e

echo "========================================="
echo "ROS Noetic and MAVROS Installation"
echo "========================================="

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$VERSION_ID" != "20.04" ]]; then
        echo "Warning: This script is for Ubuntu 20.04 (ROS Noetic)"
        echo "Your version: $VERSION_ID"
        read -p "Continue anyway? [y/N]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Setup sources.list
echo "Setting up ROS sources..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
echo "Adding ROS keys..."
if ! command -v curl &> /dev/null; then
    sudo apt install -y curl
fi
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install ROS Noetic Desktop
echo "Installing ROS Noetic (this may take a while)..."
sudo apt-get install -y ros-noetic-desktop-full

# Initialize rosdep
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Setup environment
echo "Setting up ROS environment..."
if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi
source /opt/ros/noetic/setup.bash

# Install build dependencies
echo "Installing ROS build tools..."
sudo apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    build-essential

# Install MAVROS
echo "Installing MAVROS..."
sudo apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras

# Install GeographicLib datasets
echo "Installing GeographicLib datasets for MAVROS..."
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# Build the ROS workspace
echo "Building ILS pathfinding ROS workspace..."
cd "$(dirname "$0")/../ros1_ws"
if [ ! -d "devel" ]; then
    catkin_make
fi

# Setup workspace environment
if ! grep -q "source.*ros1_ws/devel/setup.bash" ~/.bashrc; then
    echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
fi

echo ""
echo "========================================="
echo "ROS and MAVROS installed successfully!"
echo "========================================="
echo ""
echo "To activate ROS in current terminal:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source ~/ILS-ArduPilot/ros1_ws/devel/setup.bash"
echo ""
echo "Or reload your terminal for automatic activation"
echo ""
