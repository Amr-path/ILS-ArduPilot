# scripts/plan_and_fly.py

import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pathfinding.core import GridMap, PathFinder, create_ils_region

# USER CONFIGURATIONS:
CONNECTION_STRING = "127.0.0.1:14550"    # e.g., SITL UDP endpoint or serial port for real drone
TAKEOFF_ALTITUDE = 10.0                  # Target altitude (meters) for takeoff and flight
MAP_FILE = "grid_maps/example_map.png"   # Path to the map PNG file
START_CELL = (0, 0)                      # Starting grid cell (if (0,0) corresponds to home position)
END_CELL = None                          # Target grid cell; if None, use bottom-right corner of map
USE_ILS = True                           # Whether to use ILS-constrained A* or full A*
ILS_BASE_WIDTH = 0.05                    # Initial corridor width percentage (if USE_ILS=True)
ILS_MAX_EXPANSIONS = 5                   # Max expansions for corridor

def arm_and_takeoff(vehicle, target_altitude):
    """
    Arms the vehicle and flies to target_altitude (in meters) using GUIDED mode.
    """
    # Wait until the vehicle is ready to arm
    print("Connecting to vehicle...")
    while not vehicle.is_armable:
        time.sleep(1)
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Wait for arming confirmation
    while not vehicle.armed:
        time.sleep(0.5)
    print(f"Taking off to {target_altitude}m...")
    vehicle.simple_takeoff(target_altitude)
    # Wait until the vehicle reaches a safe height
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_altitude * 0.95:  # reached 95% of target
            print("Reached target altitude")
            break
        time.sleep(1)

def compress_path(path):
    """
    Compress a cell-by-cell path into a list of waypoints where direction changes.
    This reduces the number of waypoints by merging straight segments.
    """
    if not path:
        return []
    waypoints = [path[0]]
    # Track direction of last step
    last_dx = 0
    last_dy = 0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        if (dx, dy) != (last_dx, last_dy):
            # direction changed, so add previous point as a waypoint
            waypoints.append(path[i-1])
            last_dx, last_dy = dx, dy
    # Add final endpoint
    waypoints.append(path[-1])
    return waypoints

def grid_to_location(x, y, origin_lat, origin_lon, alt, cell_size=1.0):
    """
    Convert grid cell (x, y) to a LocationGlobalRelative with given origin and cell size (meters).
    Assuming x is East direction, y is South direction relative to origin.
    """
    # Calculate offset in meters
    east_offset = x * cell_size
    north_offset = -y * cell_size
    # Earth radius (approx)
    R = 6371000.0
    # Offset in degrees
    delta_lat = (north_offset / R) * (180.0 / math.pi)
    delta_lon = (east_offset / (R * math.cos(math.radians(origin_lat)))) * (180.0 / math.pi)
    target_lat = origin_lat + delta_lat
    target_lon = origin_lon + delta_lon
    return LocationGlobalRelative(target_lat, target_lon, alt)

def main():
    # Connect to the ArduPilot vehicle
    vehicle = connect(CONNECTION_STRING, wait_ready=True)
    # Arm and take off to the desired altitude
    arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)

    # Load the grid map
    grid_map = GridMap.from_file(MAP_FILE)
    pathfinder = PathFinder(grid_map)
    # Define start and end cells for pathfinding
    start_cell = START_CELL
    if END_CELL is None:
        end_cell = (grid_map.width - 1, grid_map.height - 1)
    else:
        end_cell = END_CELL

    # Plan path using A* (with or without ILS)
    if USE_ILS:
        print("Planning path using A* with Incremental Line Search...")
        corridor_width = ILS_BASE_WIDTH
        path = []
        for attempt in range(1, ILS_MAX_EXPANSIONS + 1):
            ils_region = create_ils_region(grid_map, start_cell, end_cell, corridor_width)
            path, _ = pathfinder.a_star(start_cell, end_cell, ils_region=ils_region)
            if path:
                print(f"Found path with corridor width = {corridor_width:.2f} (attempt {attempt})")
                break
            corridor_width += ILS_BASE_WIDTH
        if not path:
            # If no path found with ILS constraints, fall back to full A* search
            print("No path found within ILS corridor limits, running full A* search...")
            path, _ = pathfinder.a_star(start_cell, end_cell)
    else:
        print("Planning path using standard A* (no ILS)...")
        path, _ = pathfinder.a_star(start_cell, end_cell)

    if not path:
        print("❌ No path could be found from start to end on the map. Exiting.")
        vehicle.mode = VehicleMode("LAND")
        return

    print(f"Path found with length {len(path)} waypoints (grid steps).")
    # Compress the path to fewer waypoints (straight-line segments)
    waypoints = compress_path(path)
    print(f"Compressed path to {len(waypoints)} waypoints (straight segments).")

    # Get the drone's starting location (lat, lon)
    origin_lat = vehicle.location.global_relative_frame.lat
    origin_lon = vehicle.location.global_relative_frame.lon

    # Fly through each waypoint
    vehicle.airspeed = 2  # set a safe airspeed (m/s) for travelling
    for idx, (x, y) in enumerate(waypoints[1:], start=1):  # skip [0] because it's the start (current position)
        target_location = grid_to_location(x, y, origin_lat, origin_lon, TAKEOFF_ALTITUDE)
        print(f"Going to waypoint {idx}/{len(waypoints)-1}: grid({x},{y}) -> lat={target_location.lat:.6f}, lon={target_location.lon:.6f}")
        vehicle.simple_goto(target_location)
        # Wait until the vehicle is near the target waypoint
        while True:
            current_lat = vehicle.location.global_relative_frame.lat
            current_lon = vehicle.location.global_relative_frame.lon
            # Compute distance to target in meters (approx)
            d_lat = (target_location.lat - current_lat) * 111000.0
            d_lon = (target_location.lon - current_lon) * 111000.0 * math.cos(math.radians(current_lat))
            distance = math.sqrt(d_lat**2 + d_lon**2)
            if distance < 2.0:  # within 2 meters of waypoint
                print(f"Reached waypoint {idx}")
                break
            time.sleep(1)
        time.sleep(2)  # small pause at waypoint

    # All waypoints reached
    print("✔️ Final destination reached. Returning to launch.")
    vehicle.mode = VehicleMode("RTL")  # command Return-to-Launch (or use VehicleMode("LAND") to land in place)

    # Close vehicle object before exiting script
    vehicle.close()

if __name__ == "__main__":
    main()
