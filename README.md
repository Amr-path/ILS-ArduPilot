
# ils_pathfinding_project

Grid-based pathfinding with A*, Dijkstra, BFS, DFS, and Greedy Best-First — plus a simple ILS (Incremental Line-of-Sight) shortcutting step. Includes:

- `scripts/plan_compare.py` — run algorithms on PNG occupancy grids, compare Standard vs **+ILS**, and export plots & metrics.
- `scripts/plan_and_fly.py` — (optional) connect to ArduPilot via DroneKit, plan with A*+ILS, and fly the path (SITL or real).

> **Safety**: Always validate in SITL first. Review waypoint conversions and failsafes before any real flight.

## Install

```bash
python -m venv .venv && source .venv/bin/activate  # or use conda
pip install -r requirements.txt
```

## Maps

Put binary occupancy maps (PNG) in `grid_maps/`. Free space should be bright/white; obstacles dark/black.
An example map is included: `grid_maps/example_map.png`.

## Offline Comparison

```bash
python scripts/plan_compare.py \
  --map grid_maps/example_map.png \
  --start 5,5 --goal 90,90 \
  --algos A* Dijkstra "Best-First" BFS DFS \
  --out outputs/demo
```

Artifacts:
- `outputs/demo/paths.png` — grid with all found paths overlaid
- `outputs/demo/metrics.csv` — nodes expanded, runtime, path length, etc.

## ILS (Simplified)

This project implements a **simple ILS shortcutting** stage that compacts the waypoint list by skipping intermediate points whenever a direct line-of-sight exists (Bresenham ray casting). It is lightweight and works as a post-processor to any grid-based path.

## Fly with ArduPilot (Optional)

Requires `dronekit` and `pymavlink`.

1. Start ArduPilot SITL or connect to a vehicle that is safe to control.
2. Run:

```bash
python scripts/plan_and_fly.py \
  --map grid_maps/example_map.png \
  --start 5,5 --goal 90,90 \
  --connect udp:127.0.0.1:14550 \
  --alt 10 \
  --meters_per_cell 1.0 \
  --north_is_row_neg
```

- The script plans with A* then applies ILS shortcutting.
- It converts grid cells to GPS offsets relative to the current home position.
- The `--north_is_row_neg` flag reflects that image rows typically increase **southward**; enabling the flag means moving *up* in the image sends positive **north** offsets.

> **Important**: The meters-to-GPS conversion is approximate. Validate the mapping and scale (`--meters_per_cell`) in your environment. Tuning delays between waypoints (`time.sleep`) may be required.

## Notes

- Algorithms support 4- or 8-connectivity (BFS/DFS are best used with 4-connectivity).
- Heuristics: Euclidean and Manhattan.
- Metrics include path length (cell units), nodes expanded, runtime.
