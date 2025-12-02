#!/bin/bash
# Installation script for ILS-ArduPilot dependencies

set -e  # Exit on error

echo "========================================="
echo "ILS-ArduPilot Dependency Installation"
echo "========================================="

# Check if running on Ubuntu
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$ID" != "ubuntu" ]]; then
        echo "Warning: This script is designed for Ubuntu. You may encounter issues on $ID."
    fi
else
    echo "Warning: Cannot determine OS. Proceeding anyway..."
fi

# Update package list
echo ""
echo "Updating package list..."
sudo apt-get update

# Install system dependencies
echo ""
echo "Installing system dependencies..."
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    git \
    wget \
    curl \
    build-essential

# Create virtual environment (optional but recommended)
echo ""
read -p "Do you want to create a Python virtual environment? (recommended) [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    source venv/bin/activate
    echo "Virtual environment activated. To activate it later, run: source venv/bin/activate"
fi

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install -r requirements.txt

echo ""
echo "========================================="
echo "Core dependencies installed successfully!"
echo "========================================="

# Ask about ArduPilot SITL
echo ""
read -p "Do you want to install ArduPilot SITL for simulation? [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Installing ArduPilot SITL..."

    # Install ArduPilot prerequisites
    sudo apt-get install -y \
        git \
        gitk \
        git-gui \
        gcc-arm-none-eabi \
        python3-matplotlib \
        python3-serial \
        python3-wxgtk4.0 \
        python3-lxml \
        python3-opencv

    # Clone ArduPilot
    if [ ! -d "$HOME/ardupilot" ]; then
        echo "Cloning ArduPilot repository..."
        cd ~
        git clone https://github.com/ArduPilot/ardupilot.git
        cd ardupilot
        git submodule update --init --recursive
    else
        echo "ArduPilot already exists at ~/ardupilot"
        cd ~/ardupilot
    fi

    # Install Python requirements
    pip3 install --user -r Tools/environment_install/requirements.txt

    # Run install script
    echo "Running ArduPilot prerequisite installer..."
    Tools/environment_install/install-prereqs-ubuntu.sh -y

    # Build SITL
    echo "Building ArduCopter SITL..."
    cd ~/ardupilot
    ./waf configure --board sitl
    ./waf copter

    # Install MAVProxy
    pip3 install --user MAVProxy pymavlink

    echo ""
    echo "ArduPilot SITL installed successfully!"
    echo "Reload your profile with: source ~/.profile"
fi

# Ask about ROS
echo ""
read -p "Do you want to install ROS Noetic and MAVROS? [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    ./scripts/install_ros_mavros.sh
fi

echo ""
echo "========================================="
echo "Installation complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Read docs/QUICKSTART.md for a quick guide"
echo "2. Read docs/ARDUPILOT_SITL_SETUP.md for simulation setup"
echo "3. Try running: python3 scripts/plan_compare.py --map grid_maps/example_map.png --start 5,5 --goal 90,90"
echo ""
