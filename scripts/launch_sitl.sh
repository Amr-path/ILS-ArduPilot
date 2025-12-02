#!/bin/bash
# Launch ArduPilot SITL for ILS pathfinding testing

ARDUPILOT_DIR="$HOME/ardupilot"

# Check if ArduPilot is installed
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "Error: ArduPilot not found at $ARDUPILOT_DIR"
    echo "Please install ArduPilot first using: ./scripts/install_dependencies.sh"
    exit 1
fi

# Default parameters
LOCATION="CMAC"  # Default location
SPEEDUP=1         # Real-time simulation
CONSOLE=true
MAP=true
INSTANCE=0

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --location)
            LOCATION="$2"
            shift 2
            ;;
        --speedup)
            SPEEDUP="$2"
            shift 2
            ;;
        --no-console)
            CONSOLE=false
            shift
            ;;
        --no-map)
            MAP=false
            shift
            ;;
        --instance)
            INSTANCE="$2"
            shift 2
            ;;
        --help)
            echo "Launch ArduPilot SITL simulation"
            echo ""
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --location LOCATION    Set simulation location (default: CMAC)"
            echo "                        Common locations: CMAC, KSFO, or lat,lon,alt,heading"
            echo "  --speedup FACTOR      Simulation speed multiplier (default: 1)"
            echo "  --no-console          Disable MAVProxy console"
            echo "  --no-map              Disable map window"
            echo "  --instance NUM        SITL instance number for multi-vehicle (default: 0)"
            echo "  --help                Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Basic SITL with defaults"
            echo "  $0 --location 37.7749,-122.4194,100,0 # Custom GPS location"
            echo "  $0 --speedup 5 --no-map               # Fast simulation, no map"
            echo ""
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Run '$0 --help' for usage information"
            exit 1
            ;;
    esac
done

# Build command
CMD="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"

if [ "$CONSOLE" = true ]; then
    CMD="$CMD --console"
fi

if [ "$MAP" = true ]; then
    CMD="$CMD --map"
fi

CMD="$CMD --location $LOCATION"
CMD="$CMD --speedup $SPEEDUP"
CMD="$CMD -I $INSTANCE"

echo "========================================="
echo "Launching ArduPilot SITL"
echo "========================================="
echo "Location: $LOCATION"
echo "Speed: ${SPEEDUP}x"
echo "Instance: $INSTANCE"
echo "Connection: udp:127.0.0.1:$((14550 + 10 * INSTANCE))"
echo "========================================="
echo ""
echo "Starting SITL..."
echo "Command: $CMD"
echo ""

# Change to ArduCopter directory and run
cd "$ARDUPILOT_DIR/ArduCopter"
$CMD
