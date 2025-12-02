# ILS-ArduPilot

**Intelligent pathfinding for autonomous drones using ArduPilot**

Grid-based pathfinding with multiple algorithms (A*, Dijkstra, BFS, DFS, Greedy Best-First) enhanced with ILS (Incremental Line Search) for efficient obstacle avoidance and path planning.

## Features

- **Multiple Pathfinding Algorithms**: A*, Dijkstra, BFS, DFS, and Greedy Best-First Search
- **ILS Optimization**: Incremental Line Search corridor-based pathfinding for efficient paths
- **ArduPilot Integration**: Direct integration with ArduPilot via DroneKit and ROS/MAVROS
- **Simulation Support**: Full SITL (Software-In-The-Loop) simulation capability
- **Real Hardware Ready**: Deploy to actual drones with comprehensive safety features
- **Flexible Map Support**: PNG-based occupancy grid maps
- **Offline Analysis**: Compare algorithms and visualize paths without hardware

## Quick Start

### Installation (One Command)

```bash
git clone https://github.com/YOUR_USERNAME/ILS-ArduPilot.git
cd ILS-ArduPilot
./scripts/install_dependencies.sh
```

### Run Your First Mission (Simulation)

```bash
# Terminal 1: Start ArduPilot SITL
./scripts/launch_sitl.sh

# Terminal 2: Run ILS mission
./scripts/run_ils_mission.sh
```

That's it! Watch your drone autonomously navigate in simulation.

**New to ILS-ArduPilot?** See [Quick Start Guide](docs/QUICKSTART.md) for a 15-minute tutorial.

## Documentation

### Getting Started
- **[Quick Start Guide](docs/QUICKSTART.md)** - Get running in 15 minutes
- **[Installation Guide](docs/ARDUPILOT_SITL_SETUP.md)** - Detailed setup instructions
- **[Hardware Setup](docs/HARDWARE_SETUP.md)** - Deploy to real drones

### Safety & Operations
- **[Safety Guidelines](docs/SAFETY.md)** - **CRITICAL - Read before flying!**

### Repository Structure

```
ILS-ArduPilot/
├── config/                    # Configuration files
│   ├── mission_params.yaml    # Default parameters
│   └── scenarios/             # Pre-configured mission scenarios
├── docs/                      # Documentation
│   ├── QUICKSTART.md          # Quick start guide
│   ├── ARDUPILOT_SITL_SETUP.md # SITL setup guide
│   ├── HARDWARE_SETUP.md      # Real hardware guide
│   └── SAFETY.md              # Safety guidelines
├── grid_maps/                 # Map files (PNG)
├── pathfinding/               # Core pathfinding algorithms
│   ├── core.py                # Main pathfinding implementation
│   └── visualizer.py          # Visualization tools
├── ros1_ws/                   # ROS workspace
│   └── src/ils_pathfinding/   # ROS package with MAVROS support
├── scripts/                   # Utility scripts
│   ├── install_dependencies.sh     # Automated installation
│   ├── launch_sitl.sh             # Launch ArduPilot SITL
│   ├── run_ils_mission.sh         # Run complete ILS mission
│   ├── plan_compare.py            # Offline algorithm comparison
│   └── plan_and_fly.py            # DroneKit flight script
└── requirements.txt           # Python dependencies
```

## Usage Examples

### Offline Path Planning (No Drone)

Test and visualize pathfinding algorithms:

```bash
python3 scripts/plan_compare.py \
    --map grid_maps/example_map.png \
    --start 5,5 \
    --goal 90,90 \
    --algos "A*" "Dijkstra" "Best-First" \
    --out outputs/comparison
```

Results:
- `outputs/comparison/paths.png` - Visualized paths
- `outputs/comparison/metrics.csv` - Performance metrics

### Simulation with SITL

Run missions in ArduPilot SITL simulator:

```bash
# Start SITL
./scripts/launch_sitl.sh

# Run mission (in another terminal)
./scripts/run_ils_mission.sh \
    --map grid_maps/example_map.png \
    --start 5,5 \
    --goal 80,80 \
    --alt 10
```

### Using DroneKit (Python)

Direct Python control:

```bash
python3 scripts/plan_and_fly.py \
    --map grid_maps/example_map.png \
    --start 5,5 \
    --goal 90,90 \
    --connect udp:127.0.0.1:14550 \
    --alt 10 \
    --meters_per_cell 1.0 \
    --north_is_row_neg
```

### Using ROS/MAVROS

For ROS integration:

```bash
# Terminal 1: Start SITL
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map

# Terminal 2: Launch MAVROS
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@

# Terminal 3: Run ILS controller
cd ~/ILS-ArduPilot/ros1_ws
source devel/setup.bash
roslaunch ils_pathfinding ils_demo.launch \
    map_path:=$(pwd)/../grid_maps/example_map.png \
    goal_x:=80 goal_y:=80
```

### Real Hardware

**⚠️ WARNING: Read [SAFETY.md](docs/SAFETY.md) before flying real hardware!**

See [HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) for complete instructions.

```bash
# Connect to real drone via USB
python3 scripts/plan_and_fly.py \
    --map grid_maps/example_map.png \
    --start 0,0 \
    --goal 50,50 \
    --connect /dev/ttyACM0 \
    --alt 10
```

## Creating Custom Maps

1. Create a PNG image (grayscale):
   - **White pixels** = Free space (navigable)
   - **Black pixels** = Obstacles
   - Recommended size: 100x100 to 500x500 pixels

2. Save to `grid_maps/` directory

3. Test your map:
```bash
python3 scripts/plan_compare.py \
    --map grid_maps/your_map.png \
    --start 5,5 \
    --goal 95,95
```

## ILS Algorithm

This project implements **Incremental Line Search (ILS)** pathfinding:

1. Creates a corridor between start and goal using Bresenham's line algorithm
2. Searches for a path within this narrow corridor using A*
3. If no path found, incrementally expands the corridor width
4. Falls back to full A* if corridor approach fails

**Benefits:**
- Faster than full A* for simple environments
- More efficient waypoint generation
- Reduced computational overhead
- Configurable corridor width and expansion strategy

**Parameters** (in `config/mission_params.yaml`):
- `base_width`: Initial corridor width (0.0-1.0 as fraction of map size)
- `width_step`: Corridor expansion increment
- `max_width`: Maximum corridor width before fallback
- `max_expansions`: Max expansion attempts

## Mission Scenarios

Pre-configured scenarios in `config/scenarios/`:

- **basic_mission.yaml**: Short-range, low altitude testing
- **long_range_mission.yaml**: Extended range with higher altitude
- **precision_mission.yaml**: Low and slow for obstacle-rich environments

## Dependencies

### Core Python Packages
- numpy >= 1.22
- pillow >= 9.0
- matplotlib >= 3.6
- pandas >= 1.5

### ArduPilot Integration
- dronekit >= 2.9.2
- pymavlink >= 2.4.0

### ROS Integration (Optional)
- ROS Noetic
- MAVROS

See [requirements.txt](requirements.txt) for complete list.

## Supported Hardware

- **Flight Controllers**: Pixhawk 4+, Cube Orange/Yellow, or any ArduPilot-compatible FC
- **Companion Computers**: Raspberry Pi 4, Jetson Nano/Xavier, Intel NUC
- **Communication**: USB, telemetry radio, or WiFi

## Safety Features

- Pre-flight checks and validation
- Automatic failsafes (RTL on radio loss, low battery)
- Geofencing support
- Path validation before flight
- Emergency abort procedures
- Comprehensive logging

**Always read [SAFETY.md](docs/SAFETY.md) before operating!**

## Troubleshooting

### Common Issues

**"Connection refused" or "No heartbeat"**
- Ensure SITL/drone is running and accessible
- Check connection string matches
- Wait 10-15 seconds after SITL starts

**"No path found"**
- Verify start/goal are on free space (white pixels)
- Increase ILS corridor width
- Check map file exists and is correct format

**GPS issues on real hardware**
- Move to open area with clear sky view
- Wait 2-5 minutes for GPS lock
- Ensure 8+ satellites before flight

See individual documentation files for more troubleshooting.

## Contributing

Contributions welcome! Please:
1. Test thoroughly in SITL before submitting
2. Follow existing code style
3. Update documentation as needed
4. Add tests for new features

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

## Acknowledgments

- [ArduPilot](https://ardupilot.org/) - Open-source autopilot software
- [DroneKit](https://dronekit.io/) - Python API for vehicle control
- [MAVROS](http://wiki.ros.org/mavros) - ROS package for MAVLink communication

## Citation

If you use this project in your research, please cite:

```
@software{ils_ardupilot,
  title={ILS-ArduPilot: Intelligent Pathfinding for Autonomous Drones},
  author={Your Name},
  year={2024},
  url={https://github.com/YOUR_USERNAME/ILS-ArduPilot}
}
```

## Disclaimer

This software is provided AS-IS with NO WARRANTIES. You are solely responsible for safe operation of your drone. Always follow local laws and regulations. See [SAFETY.md](docs/SAFETY.md) for important safety information.

---

**Ready to get started? Follow the [Quick Start Guide](docs/QUICKSTART.md)!**
