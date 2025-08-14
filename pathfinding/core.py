# pathfinding/core.py

import numpy as np
import heapq
from collections import deque
from typing import List, Tuple, Dict, Set
from dataclasses import dataclass
from PIL import Image

@dataclass
class Node:
    x: int
    y: int
    g_cost: float = float('inf')
    h_cost: float = 0.0
    parent: 'Node' = None

    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost

    def __lt__(self, other: 'Node') -> bool:
        return self.f_cost < other.f_cost

class GridMap:
    def __init__(self, grid: np.ndarray):
        """
        grid: a 2D numpy array with values:
              0 -> free cell, 1 -> obstacle
        """
        self.grid = grid
        self.height, self.width = grid.shape

    @classmethod
    def from_file(cls, filepath: str) -> 'GridMap':
        """Load a grid map from a PNG file (black pixels -> obstacle, white -> free)."""
        if not filepath.lower().endswith('.png'):
            raise ValueError("Only .png files are supported for grid maps.")
        img = Image.open(filepath).convert('L')  # grayscale
        arr = np.array(img)
        # Assuming black (<128) as obstacles
        grid = (arr < 128).astype(int)
        return cls(grid)

    def is_valid_position(self, x: int, y: int) -> bool:
        """Check if a position is within bounds and not an obstacle."""
        return (0 <= x < self.width) and (0 <= y < self.height) and (self.grid[y, x] == 0)

    def get_neighbors(self, node: 'Node') -> List[Tuple[int, int]]:
        """Get valid 4-directional neighboring coordinates (up, right, down, left)."""
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            if self.is_valid_position(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

class Bresenham:
    @staticmethod
    def get_line(start: Tuple[int, int], end: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Generate points along Bresenham's line from start to end (inclusive)."""
        x1, y1 = start
        x2, y2 = end
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x2, y2))
        return points

class PathFinder:
    def __init__(self, grid_map: GridMap):
        self.grid_map = grid_map

    @staticmethod
    def manhattan_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
        """Calculate Manhattan distance between two points."""
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def a_star(self, start: Tuple[int, int], end: Tuple[int, int],
               ils_region: Set[Tuple[int, int]] = None) -> Tuple[List[Tuple[int,int]], Dict]:
        """A* pathfinding. If ils_region is provided, restrict search to that region."""
        start_node = Node(start[0], start[1], g_cost=0.0)
        start_node.h_cost = self.manhattan_distance(start, end)
        open_set = [start_node]
        closed_set = set()
        nodes = {start: start_node}
        while open_set:
            current = heapq.heappop(open_set)
            if (current.x, current.y) == end:
                # Path found - reconstruct path
                path = []
                node = current
                while node:
                    path.append((node.x, node.y))
                    node = node.parent
                path.reverse()
                metrics = {
                    'time': None,  # fill in if timing outside
                    'visited_nodes': len(closed_set),
                    'path_length': len(path)
                }
                return path, metrics
            closed_set.add((current.x, current.y))
            for (nx, ny) in self.grid_map.get_neighbors(current):
                # If using ILS, skip neighbor outside the allowed corridor
                if ils_region and (nx, ny) not in ils_region:
                    continue
                if (nx, ny) in closed_set:
                    continue
                if (nx, ny) not in nodes:
                    nodes[(nx, ny)] = Node(nx, ny)
                neighbor_node = nodes[(nx, ny)]
                tentative_g = current.g_cost + 1
                if tentative_g < neighbor_node.g_cost:
                    neighbor_node.parent = current
                    neighbor_node.g_cost = tentative_g
                    neighbor_node.h_cost = self.manhattan_distance((nx, ny), end)
                    # If not already in open_set, push it
                    if neighbor_node not in open_set:
                        heapq.heappush(open_set, neighbor_node)
        # No path found
        return [], {
            'time': None,
            'visited_nodes': len(closed_set),
            'path_length': 0
        }

    def dijkstra(self, start: Tuple[int, int], end: Tuple[int, int],
                 ils_region: Set[Tuple[int, int]] = None) -> Tuple[List[Tuple[int,int]], Dict]:
        """Dijkstra's algorithm for shortest path (no heuristic)."""
        start_node = Node(start[0], start[1], g_cost=0.0)
        open_set = [start_node]
        closed_set = set()
        nodes = {start: start_node}
        while open_set:
            current = heapq.heappop(open_set)
            if (current.x, current.y) == end:
                path = []
                node = current
                while node:
                    path.append((node.x, node.y))
                    node = node.parent
                path.reverse()
                metrics = {
                    'time': None,
                    'visited_nodes': len(closed_set),
                    'path_length': len(path)
                }
                return path, metrics
            closed_set.add((current.x, current.y))
            for (nx, ny) in self.grid_map.get_neighbors(current):
                if ils_region and (nx, ny) not in ils_region:
                    continue
                if (nx, ny) in closed_set:
                    continue
                if (nx, ny) not in nodes:
                    nodes[(nx, ny)] = Node(nx, ny)
                neighbor_node = nodes[(nx, ny)]
                tentative_g = current.g_cost + 1
                if tentative_g < neighbor_node.g_cost:
                    neighbor_node.parent = current
                    neighbor_node.g_cost = tentative_g
                    if neighbor_node not in open_set:
                        heapq.heappush(open_set, neighbor_node)
        return [], {
            'time': None,
            'visited_nodes': len(closed_set),
            'path_length': 0
        }

    def bfs(self, start: Tuple[int, int], end: Tuple[int, int],
            ils_region: Set[Tuple[int, int]] = None) -> Tuple[List[Tuple[int,int]], Dict]:
        """Breadth-First Search for pathfinding."""
        from collections import deque
        visited = {start}
        queue = deque([(start, [start])])
        visited_nodes = 0
        while queue:
            (cx, cy), path = queue.popleft()
            visited_nodes += 1
            if (cx, cy) == end:
                metrics = {
                    'time': None,
                    'visited_nodes': visited_nodes,
                    'path_length': len(path)
                }
                return path, metrics
            node = Node(cx, cy)
            for (nx, ny) in self.grid_map.get_neighbors(node):
                if ils_region and (nx, ny) not in ils_region:
                    continue
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))
        return [], {
            'time': None,
            'visited_nodes': visited_nodes,
            'path_length': 0
        }

    def dfs(self, start: Tuple[int, int], end: Tuple[int, int],
            ils_region: Set[Tuple[int, int]] = None) -> Tuple[List[Tuple[int,int]], Dict]:
        """Depth-First Search for pathfinding."""
        visited = {start}
        stack = [(start, [start])]
        visited_nodes = 0
        while stack:
            (cx, cy), path = stack.pop()
            visited_nodes += 1
            if (cx, cy) == end:
                metrics = {
                    'time': None,
                    'visited_nodes': visited_nodes,
                    'path_length': len(path)
                }
                return path, metrics
            node = Node(cx, cy)
            for (nx, ny) in self.grid_map.get_neighbors(node):
                if ils_region and (nx, ny) not in ils_region:
                    continue
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    stack.append(((nx, ny), path + [(nx, ny)]))
        return [], {
            'time': None,
            'visited_nodes': visited_nodes,
            'path_length': 0
        }

    def best_first_search(self, start: Tuple[int, int], end: Tuple[int, int],
                          ils_region: Set[Tuple[int, int]] = None) -> Tuple[List[Tuple[int,int]], Dict]:
        """Greedy Best-First Search (heuristic-guided without path cost)."""
        start_node = Node(start[0], start[1])
        start_node.h_cost = self.manhattan_distance(start, end)
        open_set = [start_node]
        closed_set = set()
        nodes = {start: start_node}
        while open_set:
            current = heapq.heappop(open_set)
            if (current.x, current.y) == end:
                path = []
                node = current
                while node:
                    path.append((node.x, node.y))
                    node = node.parent
                path.reverse()
                metrics = {
                    'time': None,
                    'visited_nodes': len(closed_set),
                    'path_length': len(path)
                }
                return path, metrics
            closed_set.add((current.x, current.y))
            for (nx, ny) in self.grid_map.get_neighbors(current):
                if ils_region and (nx, ny) not in ils_region:
                    continue
                if (nx, ny) in closed_set:
                    continue
                if (nx, ny) not in nodes:
                    nodes[(nx, ny)] = Node(nx, ny)
                neighbor_node = nodes[(nx, ny)]
                if neighbor_node not in open_set:
                    neighbor_node.parent = current
                    neighbor_node.h_cost = self.manhattan_distance((nx, ny), end)
                    heapq.heappush(open_set, neighbor_node)
        return [], {
            'time': None,
            'visited_nodes': len(closed_set),
            'path_length': 0
        }

def create_ils_region(grid_map: GridMap, start: Tuple[int, int], end: Tuple[int, int],
                      width_percentage: float) -> Set[Tuple[int, int]]:
    """
    Create a corridor of allowed cells between start and end points.
    The corridor is centered on the line from start to end (using Bresenham),
    and expands outward with a 'diamond' (Manhattan distance) shape of half-width determined by width_percentage of map size.
    """
    line = Bresenham.get_line(start, end)
    ils_region: Set[Tuple[int, int]] = set(line)
    # Compute corridor half-width in cells
    max_dim = min(grid_map.width, grid_map.height)
    half_width = int(max_dim * width_percentage)
    for (x, y) in line:
        for dx in range(-half_width, half_width + 1):
            for dy in range(-half_width, half_width + 1):
                if abs(dx) + abs(dy) <= half_width:
                    nx, ny = x + dx, y + dy
                    if grid_map.is_valid_position(nx, ny):
                        ils_region.add((nx, ny))
    return ils_region
