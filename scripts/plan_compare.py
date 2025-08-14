# scripts/plan_compare.py

import os
import pandas as pd
from pathfinding.core import GridMap, PathFinder, create_ils_region
from pathfinding.visualizer import Visualizer

def main():
    grid_folder = "grid_maps"
    results = []
    if not os.path.exists(grid_folder):
        os.makedirs(grid_folder, exist_ok=True)
        print(f"Created '{grid_folder}' directory. Add .png map files and run again.")
        return

    # List all PNG files in the grid_maps directory
    map_files = [f for f in os.listdir(grid_folder) if f.lower().endswith('.png')]
    if not map_files:
        print(f"No PNG map files found in '{grid_folder}'.")
        return

    for filename in map_files:
        filepath = os.path.join(grid_folder, filename)
        print(f"\nProcessing map: {filename}")
        try:
            grid_map = GridMap.from_file(filepath)
        except Exception as e:
            print(f"Error loading map {filename}: {e}")
            continue

        pathfinder = PathFinder(grid_map)
        # Define start and end points (here using top-left and bottom-right corners as an example)
        start = (0, 0)
        end = (grid_map.width - 1, grid_map.height - 1)
        # Initial ILS corridor width (as a percentage of map size)
        base_width_pct = 0.05

        # Precompute the broadest ILS region we'll possibly use (for visualization after loop)
        max_ils_region = None

        # Iterate over each algorithm to test
        algorithms = [
            ("A*", pathfinder.a_star),
            ("Dijkstra", pathfinder.dijkstra),
            ("BFS", pathfinder.bfs),
            ("DFS", pathfinder.dfs),
            ("Best-First", pathfinder.best_first_search)
        ]
        for algo_name, algo_func in algorithms:
            # Run Standard version (no ILS region constraint)
            print(f"Running {algo_name} (Standard)...")
            path_std, metrics_std = algo_func(start, end)
            metrics_std.update({
                "algorithm": f"{algo_name} (Standard)",
                "map": filename,
                "start_x": start[0], "start_y": start[1],
                "end_x": end[0], "end_y": end[1],
                "corridor_width": None
            })
            results.append(metrics_std)

            # Run ILS version (incrementally expand corridor until path is found or attempts exhausted)
            print(f"Running {algo_name} (ILS)...")
            max_expansions = 5
            path_ils = []
            corridor_width = base_width_pct
            for attempt in range(1, max_expansions + 1):
                ils_region = create_ils_region(grid_map, start, end, corridor_width)
                if attempt == max_expansions:
                    # Keep the widest region for later visualization
                    max_ils_region = ils_region
                path_ils, metrics_ils = algo_func(start, end, ils_region)
                if path_ils:
                    print(f" ✔️ Path found with ILS at corridor width = {corridor_width:.2f}")
                    break
                else:
                    print(f" ❌ No path at width = {corridor_width:.2f}, expanding...")
                    corridor_width += base_width_pct  # expand corridor
            if not path_ils:
                print(f" ⚠️  No path found for {algo_name} even after maximum ILS expansions.")
                metrics_ils = {}  # ensure metrics_ils exists even if not set
            # Record ILS metrics
            metrics_ils.update({
                "algorithm": f"{algo_name} (ILS)",
                "map": filename,
                "start_x": start[0], "start_y": start[1],
                "end_x": end[0], "end_y": end[1],
                "corridor_width": corridor_width
            })
            results.append(metrics_ils)

            # Visualize paths side-by-side for this algorithm
            Visualizer.plot_side_by_side(grid_map.grid, path_std, path_ils,
                                         ils_region=(max_ils_region if path_ils else None),
                                         algo_name=algo_name, map_name=filename,
                                         ax_titles=("Standard", "ILS"))
    # End of processing all maps

    if not results:
        print("No results to analyze.")
        return

    # Save results to CSV
    results_df = pd.DataFrame(results)
    results_df.to_csv("pathfinding_results.csv", index=False)
    print(f"\nAll results saved to 'pathfinding_results.csv'.")
    # Generate comparison plots and display summary table
    Visualizer.plot_comparison_metrics(results_df)
    Visualizer.display_comparison_table(results_df)
    print("Detailed plots and summary table are saved in the 'analysis_plots/' folder.")

if __name__ == "__main__":
    main()
