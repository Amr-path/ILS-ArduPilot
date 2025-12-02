# Quick Start Guide

Get up and running with ILS pathfinding in 15 minutes!

## Prerequisites

- Ubuntu 20.04 or later
- Python 3.8+
- 8GB+ RAM recommended
- Internet connection for initial setup

## Installation (5 minutes)

### Quick Install

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/ILS-ArduPilot.git
cd ILS-ArduPilot

# Run automated installation
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

Follow the prompts:
- Create Python virtual environment? → **Yes** (recommended)
- Install ArduPilot SITL? → **Yes** (for simulation)
- Install ROS and MAVROS? → Optional (choose **No** for quick start)

### Manual Install (Alternative)

```bash
# Install Python dependencies only
pip3 install -r requirements.txt
```

## Test Pathfinding Offline (2 minutes)

Test the ILS algorithm without any drone:

```bash
# Run path comparison on example map
python3 scripts/plan_compare.py \
    --map grid_maps/example_map.png \
    --start 5,5 \
    --goal 90,90 \
    --algos "A*" \
    --out outputs/test

# View results
ls outputs/test/
# You should see: paths.png and metrics.csv
```

Open `outputs/test/paths.png` to see the planned path!

## Run in Simulation (5 minutes)

### Terminal 1: Start SITL

```bash
# Launch ArduPilot simulator
./scripts/launch_sitl.sh
```

Wait for the simulation to start. You should see:
```
APM: EKF2 IMU0 is using GPS
APM: EKF2 IMU1 is using GPS
```

### Terminal 2: Run ILS Mission

```bash
# Run ILS pathfinding mission
./scripts/run_ils_mission.sh
```

This will:
1. Connect to the simulator
2. Plan a path using A* + ILS
3. Arm and takeoff
4. Fly the planned waypoints
5. Return to launch

**Watch the MAVProxy map window to see the drone fly!**

## Understanding the Output

During mission execution, you'll see:

```
Arming motors...
Taking off to 10m...
Reached target altitude
Planning path using A* with Incremental Line Search...
Found path with corridor width = 0.05 (attempt 1)
Path found with length 142 waypoints (grid steps).
Compressed path to 8 waypoints (straight segments).
Going to waypoint 1/7: grid(10,8) -> lat=-35.362778, lon=149.164321
Reached waypoint 1
...
Final destination reached. Returning to launch.
```

## Customize Your Mission

### Change Map and Goal

```bash
./scripts/run_ils_mission.sh \
    --map grid_maps/example_map.png \
    --start 10,10 \
    --goal 80,80 \
    --alt 15
```

### Use Different Scenarios

```bash
# Basic short mission
./scripts/run_ils_mission.sh --alt 5 --goal 30,30

# Long range mission
./scripts/run_ils_mission.sh --alt 20 --goal 90,90

# Precision low altitude
./scripts/run_ils_mission.sh --alt 3 --meters_per_cell 0.5
```

## Create Your Own Map

1. **Create a PNG image** (grayscale)
   - White pixels = free space
   - Black pixels = obstacles
   - Recommended size: 100x100 to 500x500 pixels

2. **Save to `grid_maps/` directory**
   ```bash
   cp your_map.png grid_maps/my_custom_map.png
   ```

3. **Test it**
   ```bash
   python3 scripts/plan_compare.py \
       --map grid_maps/my_custom_map.png \
       --start 5,5 \
       --goal 95,95 \
       --algos "A*"
   ```

4. **Fly it in SITL**
   ```bash
   ./scripts/run_ils_mission.sh \
       --map grid_maps/my_custom_map.png \
       --goal 95,95
   ```

## Using ROS/MAVROS (Optional)

If you installed ROS during setup:

### Terminal 1: Start SITL
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map -I 0
```

### Terminal 2: Start MAVROS
```bash
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@
```

### Terminal 3: Run ILS Controller
```bash
cd ~/ILS-ArduPilot/ros1_ws
source devel/setup.bash
roslaunch ils_pathfinding ils_demo.launch \
    map_path:=$(pwd)/../grid_maps/example_map.png \
    goal_x:=80 \
    goal_y:=80 \
    resolution:=1.0
```

## Troubleshooting

### "Connection refused" or "No heartbeat"
- Make sure SITL is running first
- Wait 10-15 seconds after SITL starts before connecting
- Check connection string matches (default: `udp:127.0.0.1:14550`)

### "No path found"
- Check start and goal are on white (free) space, not black (obstacle)
- Try increasing ILS corridor width
- Verify map file exists and is readable

### SITL won't start
- Make sure you installed ArduPilot SITL (`./scripts/install_dependencies.sh`)
- Try: `source ~/.profile` to reload environment
- Check Python version: `python3 --version` (needs 3.8+)

### Path looks wrong on map
- Adjust `--meters_per_cell` parameter
- Try `--south_is_row_neg` flag to flip coordinate system
- Verify map scale matches real-world units

## Example Missions

### Mission 1: Simple Navigation
```bash
./scripts/run_ils_mission.sh \
    --map grid_maps/example_map.png \
    --start 5,5 \
    --goal 50,50 \
    --alt 10
```

### Mission 2: Long Range
```bash
./scripts/run_ils_mission.sh \
    --map grid_maps/example_map.png \
    --start 10,10 \
    --goal 90,90 \
    --alt 20 \
    --meters_per_cell 2.0
```

### Mission 3: Precision Low Altitude
```bash
./scripts/run_ils_mission.sh \
    --map grid_maps/example_map.png \
    --start 15,15 \
    --goal 45,45 \
    --alt 3 \
    --meters_per_cell 0.25
```

## Next Steps

Now that you have the basics working:

1. **Read the full documentation**
   - [ARDUPILOT_SITL_SETUP.md](ARDUPILOT_SITL_SETUP.md) - Detailed simulation setup
   - [HARDWARE_SETUP.md](HARDWARE_SETUP.md) - Real drone deployment
   - [SAFETY.md](SAFETY.md) - **CRITICAL** safety guidelines

2. **Experiment with parameters**
   - See `config/mission_params.yaml` for all available options
   - Try different ILS corridor widths
   - Test various algorithms (A*, Dijkstra, etc.)

3. **Create custom scenarios**
   - Make your own maps
   - Design realistic flight paths
   - Test in different environments

4. **Deploy to real hardware** (only after thorough testing!)
   - Follow [HARDWARE_SETUP.md](HARDWARE_SETUP.md)
   - **READ [SAFETY.md](SAFETY.md) FIRST!**
   - Start with small, safe tests

## Quick Reference

### Common Commands

```bash
# Install dependencies
./scripts/install_dependencies.sh

# Start SITL
./scripts/launch_sitl.sh

# Run mission
./scripts/run_ils_mission.sh

# Test pathfinding offline
python3 scripts/plan_compare.py --map grid_maps/example_map.png --start 5,5 --goal 90,90

# Custom mission
./scripts/run_ils_mission.sh --map grid_maps/example_map.png --goal 80,80 --alt 15
```

### File Locations

- **Maps:** `grid_maps/` - Add your PNG maps here
- **Scripts:** `scripts/` - Executable scripts
- **Config:** `config/` - Parameter and scenario files
- **Docs:** `docs/` - All documentation
- **ROS:** `ros1_ws/` - ROS workspace (if using MAVROS)
- **Outputs:** `outputs/` - Generated visualizations and data

### Connection Strings

- **SITL (default):** `udp:127.0.0.1:14550`
- **Serial USB:** `/dev/ttyACM0` or `/dev/ttyUSB0`
- **TCP:** `tcp:127.0.0.1:5760`
- **Multiple vehicles:** `udp:127.0.0.1:14560` (instance 1), `14570` (instance 2), etc.

## Getting Help

- Check [Troubleshooting](#troubleshooting) section above
- Review documentation in `docs/` directory
- Check ArduPilot forums: https://discuss.ardupilot.org/
- Review logs in Mission Planner or MAVProxy

---

**Happy flying! Remember to always prioritize safety! 🚁**
