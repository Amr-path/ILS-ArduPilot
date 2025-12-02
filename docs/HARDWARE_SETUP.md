# Hardware Setup Guide

This guide covers setting up ILS pathfinding on a real drone with ArduPilot flight controller.

## ⚠️ SAFETY WARNING

**ALWAYS** test thoroughly in SITL before deploying to real hardware. Real drones can cause injury or property damage if not properly configured and tested.

## Hardware Requirements

### Minimum Requirements

1. **Flight Controller**
   - Pixhawk 4 or later
   - Cube Orange/Yellow/Purple
   - Holybro Kakute F7 AIO
   - Or any ArduPilot-compatible flight controller

2. **Companion Computer** (for running ILS pathfinding)
   - Raspberry Pi 4 (4GB+ RAM recommended)
   - NVIDIA Jetson Nano/Xavier
   - Intel NUC or similar x86 mini PC
   - Any Linux-capable computer with USB/Serial

3. **Communication**
   - Telemetry radio (RFD900x, SiK radio, etc.)
   - OR USB cable for direct connection
   - OR WiFi telemetry module

4. **GPS Module**
   - ArduPilot-compatible GPS (UBlox M8N or better)
   - Required for outdoor flight

5. **Power**
   - Battery for drone (LiPo 3S-6S depending on drone)
   - Separate power for companion computer (or BEC from main battery)

### Recommended Additional Hardware

- Obstacle avoidance sensors (for real-time obstacle detection)
- Optical flow sensor (for indoor GPS-denied operation)
- RC transmitter with failsafe capability
- Ground control station tablet/laptop

## Software Setup

### 1. Flash ArduPilot to Flight Controller

Use Mission Planner, QGroundControl, or ArduPilot Web Tools:

```bash
# Using Mission Planner (Windows)
# 1. Connect flight controller via USB
# 2. Install Firmware → ArduCopter → Latest stable version

# Or use command line (Linux)
./Tools/scripts/uploader.py --port /dev/ttyACM0 ArduCopter.px4
```

### 2. Configure Companion Computer

#### Option A: DroneKit Setup (Simpler)

```bash
# On companion computer (e.g., Raspberry Pi)
sudo apt-get update
sudo apt-get install -y python3 python3-pip git

# Clone this repository
cd ~
git clone https://github.com/YOUR_USERNAME/ILS-ArduPilot.git
cd ILS-ArduPilot

# Install dependencies
pip3 install -r requirements.txt

# Test connection (flight controller should be connected)
python3 -c "from dronekit import connect; v = connect('/dev/ttyACM0', wait_ready=True); print('Connected!'); v.close()"
```

#### Option B: ROS/MAVROS Setup (Advanced)

```bash
# Install ROS and MAVROS
cd ~/ILS-ArduPilot
./scripts/install_ros_mavros.sh

# Build workspace
cd ros1_ws
catkin_make
source devel/setup.bash
```

### 3. Configure Serial Connection

#### Find Serial Port

```bash
# List connected devices
ls -l /dev/ttyACM* /dev/ttyUSB*

# Common ports:
# /dev/ttyACM0 - USB connection to Pixhawk
# /dev/ttyUSB0 - FTDI serial adapter
# /dev/serial0 - Raspberry Pi GPIO UART
```

#### Set Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot or logout/login for changes to take effect
```

#### Configure MAVLink Stream Rate

In Mission Planner or via MAVProxy:
```
# Set stream rate (Hz)
SERIAL1_BAUD 921600    # For USB/Telemetry1
SR0_EXTRA1 10          # Attitude data
SR0_EXTRA2 10          # VFR_HUD
SR0_EXTRA3 2           # System time
SR0_POSITION 10        # GPS position
SR0_RAW_SENS 2         # IMU
SR0_RC_CHAN 2          # RC channels
```

## Physical Connections

### USB Connection
```
Flight Controller USB ←→ Companion Computer USB Port
```
**Connection String:** `/dev/ttyACM0` or `/dev/ttyUSB0`

### Telemetry Radio
```
Flight Controller TELEM1/2 Port ←→ Telemetry Radio ←→ Companion Computer USB
```
**Connection String:** `/dev/ttyUSB0` (baud: 57600 or 921600)

### Serial UART (Raspberry Pi)
```
Flight Controller TELEM Port ←→ RPi GPIO UART (TX→RX, RX→TX, GND→GND)
```
**Connection String:** `/dev/serial0` (baud: 921600)
**Note:** Disable console on serial: `sudo raspi-config` → Interface → Serial → Disable console, Enable hardware

## Pre-Flight Configuration

### 1. Basic ArduPilot Configuration

Use Mission Planner or QGroundControl:

1. **Calibrate Sensors**
   - Accelerometer calibration
   - Compass calibration
   - Radio calibration
   - ESC calibration

2. **Set Flight Modes**
   - Mode 1: Stabilize
   - Mode 2: Alt Hold
   - Mode 3: Loiter
   - Mode 4: RTL
   - Mode 5: Auto (for missions)
   - Mode 6: GUIDED (required for ILS pathfinding)

3. **Configure Failsafes**
   - Radio failsafe → RTL
   - Battery failsafe → RTL (at 20%)
   - GCS failsafe → RTL (after 30s)
   - EKF failsafe → Land

4. **Set Parameters**
   ```
   # Enable GUIDED mode for DroneKit
   SYSID_MYGCS 1

   # Geofencing (optional but recommended)
   FENCE_ENABLE 1
   FENCE_TYPE 3          # Cylinder
   FENCE_RADIUS 500      # 500m radius
   FENCE_ALT_MAX 120     # 120m max altitude

   # Battery monitoring
   BATT_MONITOR 4        # Analog voltage and current
   BATT_CAPACITY 5000    # mAh
   BATT_LOW_VOLT 10.5    # Low battery voltage
   BATT_CRT_VOLT 9.8     # Critical voltage
   ```

### 2. Test Manual Flight

**CRITICAL:** Always test manual flight first!

1. Perform pre-flight checks
2. Test in Stabilize mode
3. Test in Alt Hold mode
4. Test in Loiter mode
5. Test RTL functionality
6. Verify GPS lock and home position

### 3. Test GUIDED Mode

```bash
# On companion computer, create test script
python3 << 'EOF'
from dronekit import connect, VehicleMode
import time

# Connect to vehicle
vehicle = connect('/dev/ttyACM0', wait_ready=True)

print("Connected to vehicle")
print("Mode:", vehicle.mode.name)
print("Armed:", vehicle.armed)
print("GPS:", vehicle.gps_0)
print("Battery:", vehicle.battery)

# Try switching to GUIDED (on ground)
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)
print("Mode after switch:", vehicle.mode.name)

vehicle.close()
EOF
```

## Running ILS Pathfinding on Real Hardware

### Safety Checklist

- [ ] All manual flight modes tested
- [ ] GPS lock acquired (3D fix, >8 satellites)
- [ ] Compass calibrated and healthy
- [ ] Battery fully charged and monitored
- [ ] Failsafes configured and tested
- [ ] Geofence enabled (if applicable)
- [ ] Clear flight area with no people nearby
- [ ] RC transmitter ready with failsafe mode switch
- [ ] Emergency stop procedure planned
- [ ] Weather conditions suitable (low wind, good visibility)

### Method 1: DroneKit

```bash
cd ~/ILS-ArduPilot

# Edit connection string in scripts/plan_and_fly.py
# Change line 9: CONNECTION_STRING = "/dev/ttyACM0"

# Or run with custom connection:
python3 scripts/plan_and_fly.py \
    --map grid_maps/example_map.png \
    --start 0,0 \
    --goal 50,50 \
    --connect /dev/ttyACM0 \
    --alt 10 \
    --meters_per_cell 1.0
```

### Method 2: ROS/MAVROS

```bash
# Terminal 1: Launch MAVROS
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600

# Terminal 2: Launch ILS controller
cd ~/ILS-ArduPilot/ros1_ws
source devel/setup.bash
roslaunch ils_pathfinding ils_demo.launch \
    map_path:=$(pwd)/../grid_maps/example_map.png \
    goal_x:=50 \
    goal_y:=50
```

### Emergency Procedures

1. **Manual Override:** Switch to Stabilize or Alt Hold mode on RC transmitter
2. **Return to Launch:** Switch to RTL mode
3. **Emergency Stop:** Disarm motors (not recommended while airborne)
4. **Kill Script:** Ctrl+C on companion computer terminal

## Troubleshooting

### Connection Issues

```bash
# Check if device is detected
lsusb
ls -l /dev/ttyACM* /dev/ttyUSB*

# Check permissions
groups $USER  # should include 'dialout'

# Test connection
python3 -c "from dronekit import connect; connect('/dev/ttyACM0', wait_ready=True)"
```

### GPS Issues
- Move to open area away from buildings
- Wait 2-5 minutes for GPS lock
- Check compass interference (keep away from metal/magnets)

### Mode Switch Failures
- Ensure GPS lock before switching to GUIDED
- Check EKF status (must be healthy)
- Verify all pre-arm checks pass

### Path Planning Failures
- Verify map file exists and is correct format
- Check start/goal positions are valid (not on obstacles)
- Increase ILS corridor width if no path found

## Best Practices

1. **Always Start Conservative**
   - Low altitude first (2-3m)
   - Short distances
   - Slow speeds
   - Good weather

2. **Incremental Testing**
   - Test single waypoint navigation
   - Test short paths before long paths
   - Gradually increase complexity

3. **Monitor During Flight**
   - Battery voltage
   - GPS lock status
   - Distance from home
   - Current mode

4. **Post-Flight Analysis**
   - Download logs from flight controller
   - Review in Mission Planner
   - Check for errors or warnings

## Next Steps

- Read [SAFETY.md](SAFETY.md) for comprehensive safety guidelines
- Review [QUICKSTART.md](QUICKSTART.md) for mission examples
- See configuration files in `config/` directory for parameter tuning
