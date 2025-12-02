#!/bin/bash
# Run a complete ILS pathfinding mission with ArduPilot SITL

set -e

# Default parameters
MAP="grid_maps/example_map.png"
START="5,5"
GOAL="90,90"
CONNECTION="udp:127.0.0.1:14550"
ALTITUDE=10
METERS_PER_CELL=1.0
NORTH_IS_ROW_NEG="--north_is_row_neg"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --map)
            MAP="$2"
            shift 2
            ;;
        --start)
            START="$2"
            shift 2
            ;;
        --goal)
            GOAL="$2"
            shift 2
            ;;
        --connect)
            CONNECTION="$2"
            shift 2
            ;;
        --alt)
            ALTITUDE="$2"
            shift 2
            ;;
        --meters_per_cell)
            METERS_PER_CELL="$2"
            shift 2
            ;;
        --south_is_row_neg)
            NORTH_IS_ROW_NEG=""
            shift
            ;;
        --help)
            echo "Run ILS pathfinding mission with ArduPilot"
            echo ""
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --map FILE             Path to map PNG file (default: grid_maps/example_map.png)"
            echo "  --start X,Y            Start grid cell coordinates (default: 5,5)"
            echo "  --goal X,Y             Goal grid cell coordinates (default: 90,90)"
            echo "  --connect STRING       Connection string (default: udp:127.0.0.1:14550)"
            echo "  --alt METERS           Flight altitude in meters (default: 10)"
            echo "  --meters_per_cell M    Map scale in meters per cell (default: 1.0)"
            echo "  --south_is_row_neg     Use if image rows increase northward (default: rows increase southward)"
            echo "  --help                 Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                          # Use all defaults"
            echo "  $0 --map my_map.png --goal 100,100         # Custom map and goal"
            echo "  $0 --alt 20 --meters_per_cell 2.0          # Higher altitude and larger cells"
            echo ""
            echo "Note: Make sure SITL is running first:"
            echo "  ./scripts/launch_sitl.sh"
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

# Check if map exists
if [ ! -f "$MAP" ]; then
    echo "Error: Map file not found: $MAP"
    exit 1
fi

echo "========================================="
echo "ILS Pathfinding Mission"
echo "========================================="
echo "Map: $MAP"
echo "Start: $START"
echo "Goal: $GOAL"
echo "Connection: $CONNECTION"
echo "Altitude: ${ALTITUDE}m"
echo "Scale: ${METERS_PER_CELL}m per cell"
echo "========================================="
echo ""
echo "Waiting 3 seconds before starting mission..."
sleep 3
echo "Starting mission..."
echo ""

# Run the mission
python3 scripts/plan_and_fly.py \
    --map "$MAP" \
    --start "$START" \
    --goal "$GOAL" \
    --connect "$CONNECTION" \
    --alt "$ALTITUDE" \
    --meters_per_cell "$METERS_PER_CELL" \
    $NORTH_IS_ROW_NEG

echo ""
echo "Mission complete!"
