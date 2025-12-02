# ArduPilot SITL Setup Guide

This guide will help you set up ArduPilot Software-In-The-Loop (SITL) simulation to test the ILS pathfinding algorithm.

## Prerequisites

- Ubuntu 20.04 or later (recommended)
- Python 3.8+
- Git

## Installation Steps

### 1. Install ArduPilot SITL

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y git python3-pip python3-dev

# Clone ArduPilot repository
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install required Python packages
pip3 install --user -r Tools/environment_install/requirements.txt

# Run the install script
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload PATH
. ~/.profile
```

### 2. Build ArduCopter SITL

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

### 3. Install MAVProxy

```bash
pip3 install --user MAVProxy pymavlink
```

### 4. Install Project Dependencies

```bash
cd ~/ILS-ArduPilot
pip3 install -r requirements.txt
```

## Running SITL

### Basic SITL Launch

Start the ArduPilot SITL simulator:

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map
```

This will:
- Start the ArduCopter SITL simulator
- Launch MAVProxy console
- Open a map window
- Listen on UDP port 14550 (default connection string: `127.0.0.1:14550`)

### Launch with Custom Parameters

Start SITL with specific location and parameters:

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py \
  --console \
  --map \
  --location CMAC \
  --speedup 1
```

Parameters:
- `--location`: Pre-defined location (CMAC, KSFO, etc.) or custom lat,lon,alt,heading
- `--speedup`: Simulation speed multiplier (1 = real-time)
- `--console`: Enable MAVProxy console
- `--map`: Enable map display

### Custom Location

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py \
  --console \
  --map \
  --location 37.7749,-122.4194,100,0
```

## Testing the ILS Algorithm with SITL

### Method 1: Using DroneKit Script

1. Start SITL in one terminal:
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map
```

2. In another terminal, run the ILS pathfinding script:
```bash
cd ~/ILS-ArduPilot
python3 scripts/plan_and_fly.py \
  --map grid_maps/example_map.png \
  --start 5,5 \
  --goal 90,90 \
  --connect udp:127.0.0.1:14550 \
  --alt 10 \
  --meters_per_cell 1.0 \
  --north_is_row_neg
```

### Method 2: Using ROS with MAVROS

1. Install ROS and MAVROS (if not already installed):
```bash
# Follow the installation script in scripts/install_ros_mavros.sh
./scripts/install_ros_mavros.sh
```

2. Start SITL:
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map -I 0
```

3. Launch MAVROS:
```bash
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@
```

4. In another terminal, build and run the ROS package:
```bash
cd ~/ILS-ArduPilot/ros1_ws
catkin_make
source devel/setup.bash
roslaunch ils_pathfinding ils_demo.launch \
  map_path:=$(pwd)/../grid_maps/example_map.png \
  goal_x:=80 \
  goal_y:=80
```

## SITL Connection Strings

- UDP (default): `udp:127.0.0.1:14550`
- TCP: `tcp:127.0.0.1:5760`
- Serial: `/dev/ttyUSB0` or `/dev/ttyACM0`

## Troubleshooting

### SITL won't start
- Ensure all dependencies are installed
- Try cleaning and rebuilding: `./waf clean && ./waf copter`
- Check Python version: `python3 --version` (should be 3.8+)

### Connection refused
- Verify SITL is running and listening on the correct port
- Check firewall settings
- Ensure correct connection string in your script

### Vehicle won't arm
- Check GPS lock (SITL usually gets instant GPS lock)
- Verify mode is set to GUIDED
- Check for pre-arm failures in MAVProxy console

### Path not found
- Verify map file exists and is readable
- Check start and goal coordinates are within map bounds
- Ensure start and goal are not on obstacles (black pixels)

## Next Steps

- See [QUICKSTART.md](QUICKSTART.md) for a quick guide to running missions
- See [HARDWARE_SETUP.md](HARDWARE_SETUP.md) for real drone deployment
- See [SAFETY.md](SAFETY.md) for safety guidelines and best practices
