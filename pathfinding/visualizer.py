# pathfinding/visualizer.py

import os
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

class Visualizer:
    @staticmethod
    def plot_side_by_side(grid, path_standard, path_ils, ils_region, algo_name, map_name,
                          ax_titles=("Standard", "Incremental Line Search")):
        """
        Plot a side-by-side comparison of Standard vs. ILS paths for a given algorithm and map.
        """
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        # Left: Standard path
        Visualizer.plot_grid_with_path(grid, path_standard, ils_region=None,
                                       title=f"{algo_name} - {ax_titles[0]}", ax=axes[0])
        # Right: ILS path (corridor highlighted)
        Visualizer.plot_grid_with_path(grid, path_ils, ils_region=ils_region,
                                       title=f"{algo_name} - {ax_titles[1]}", ax=axes[1])
        fig.suptitle(f"{algo_name} on {map_name}", fontsize=14)
        plt.tight_layout()
        # Save figure
        os.makedirs("analysis_plots", exist_ok=True)
        filename = f"{map_name}_{algo_name}_comparison.png".replace(".png", "_compare.png")
        save_path = os.path.join("analysis_plots", filename)
        plt.savefig(save_path, dpi=150)
        plt.show()

    @staticmethod
    def plot_grid_with_path(grid, path, ils_region=None, title="", ax=None):
        """Helper to plot a single grid with a path (and optional ILS region highlighted)."""
        if ax is None:
            _, ax = plt.subplots()
        ax.imshow(grid, cmap='binary')  # 0=white (free), 1=black (obstacle)
        if ils_region:
            # Plot ILS region as semi-transparent yellow overlay
            rr_y, rr_x = zip(*ils_region)
            ax.scatter(rr_x, rr_y, color='yellow', alpha=0.3, s=30, label='ILS Region')
        if path:
            # path is a list of (x,y) pairs; plot as line on grid (note: matplotlib image axes: y->row, x->col)
            path_y, path_x = zip(*[(y, x) for (x, y) in path])
            ax.plot(path_x, path_y, 'r-', linewidth=2, label='Path')
        ax.set_title(title)
        ax.grid(True)
        ax.legend()

    @staticmethod
    def plot_comparison_metrics(results_df: pd.DataFrame):
        """
        Create bar plots for time, visited_nodes, and path_length comparisons (Standard vs ILS).
        """
        os.makedirs("analysis_plots", exist_ok=True)
        metrics = ['time', 'visited_nodes', 'path_length']
        # Add an 'approach' column to distinguish Standard vs ILS in the dataframe
        df_comp = results_df.copy()
        df_comp['approach'] = df_comp['algorithm'].apply(lambda x: 'ILS' if 'ILS' in x else 'Standard')
        df_comp['algorithm'] = (df_comp['algorithm'].str.replace(' (Standard)', '', regex=False)
                                                .str.replace(' (ILS)', '', regex=False))
        plt.figure(figsize=(10, 5 * len(metrics)))
        for i, metric in enumerate(metrics, start=1):
            plt.subplot(len(metrics), 1, i)
            sns.barplot(data=df_comp, x='algorithm', y=metric, hue='approach', dodge=True)
            plt.title(f"{metric.replace('_', ' ').title()} Comparison")
            plt.xticks(rotation=45)
            plt.legend(loc='best')
        plt.tight_layout()
        # Save the metrics comparison plot
        plt.savefig(os.path.join("analysis_plots", "metrics_comparison.png"), dpi=150)

    @staticmethod
    def display_comparison_table(results_df: pd.DataFrame):
        """Print and save a summary table comparing algorithm performance (Standard vs ILS)."""
        summary = results_df.groupby(['map', 'algorithm'])[['time', 'visited_nodes', 'path_length']].mean()
        print("\n--- Performance Comparison (averages) ---")
        print(summary)
        os.makedirs("analysis_plots", exist_ok=True)
        filepath = os.path.join("analysis_plots", "performance_summary_table.txt")
        with open(filepath, 'w') as f:
            f.write("--- Performance Comparison (averages) ---\n")
            f.write(summary.to_string())
        print(f"\nSummary table saved to {filepath}")
