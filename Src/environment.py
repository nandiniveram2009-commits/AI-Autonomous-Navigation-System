"""
src/environment.py
Manages the 2D virtual world, static obstacle grid, safety inflation,
and moving dynamic obstacles (other warehouse AGVs or pedestrians).
"""

import numpy as np
from src.config import (
    GRID_COLS, GRID_ROWS, GRID_SIZE,
    SAFETY_MARGIN_CELLS, DYNAMIC_OBSTACLE_SPEED
)


class DynamicObstacle:
    """Represents a moving obstacle (e.g., pedestrian or secondary AGV)."""
    def __init__(self, x: float, y: float, vx: float, vy: float, radius: float = 12.0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = radius

    def step(self, static_grid: np.ndarray):
        """Updates position and bounces off boundary walls or static obstacles."""
        next_x = self.x + self.vx
        next_y = self.y + self.vy

        col = int(next_x // GRID_SIZE)
        row = int(next_y // GRID_SIZE)

        if col <= 1 or col >= GRID_COLS - 2 or static_grid[row, col] == 1:
            self.vx = -self.vx
        else:
            self.x = next_x

        if row <= 1 or row >= GRID_ROWS - 2 or static_grid[row, col] == 1:
            self.vy = -self.vy
        else:
            self.y = next_y


class VirtualEnvironment:
    """Maintains the static occupancy grid, inflated C-space grid, and dynamic agents."""
    def __init__(self, preset: str = "warehouse"):
        self.static_grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8)
        self.inflated_grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8)
        self.dynamic_obstacles: list[DynamicObstacle] = []
        self.start_cell = (3, 3)
        self.goal_cell = (GRID_ROWS - 5, GRID_COLS - 5)
        self.load_preset(preset)

    def load_preset(self, preset: str = "warehouse"):
        """Loads a structured warehouse or urban layout."""
        self.static_grid.fill(0)
        self.static_grid[0, :] = 1
        self.static_grid[-1, :] = 1
        self.static_grid[:, 0] = 1
        self.static_grid[:, -1] = 1

        if preset == "warehouse":
            for r in range(6, 15):
                self.static_grid[r, 10:13] = 1
                self.static_grid[r, 20:23] = 1
                self.static_grid[r, 30:33] = 1
            for r in range(23, 33):
                self.static_grid[r, 8:11] = 1
                self.static_grid[r, 18:21] = 1
                self.static_grid[r, 28:31] = 1
            self.static_grid[18:20, 14:27] = 1

        self._spawn_dynamic_obstacles()
        self.update_inflated_grid()

    def _spawn_dynamic_obstacles(self):
        self.dynamic_obstacles = [
            DynamicObstacle(320.0, 320.0, DYNAMIC_OBSTACLE_SPEED, 0.0),
            DynamicObstacle(500.0, 420.0, 0.0, DYNAMIC_OBSTACLE_SPEED),
            DynamicObstacle(220.0, 620.0, DYNAMIC_OBSTACLE_SPEED, 0.0),
            DynamicObstacle(680.0, 260.0, 0.0, -DYNAMIC_OBSTACLE_SPEED),
        ]

    def update_inflated_grid(self):
        """Computes Configuration Space (C-Space) obstacle inflation."""
        self.inflated_grid = self.static_grid.copy()
        rows, cols = np.where(self.static_grid == 1)
        for r, c in zip(rows, cols):
            r_min = max(0, r - SAFETY_MARGIN_CELLS)
            r_max = min(GRID_ROWS - 1, r + SAFETY_MARGIN_CELLS)
            c_min = max(0, c - SAFETY_MARGIN_CELLS)
            c_max = min(GRID_COLS - 1, c + SAFETY_MARGIN_CELLS)
            self.inflated_grid[r_min:r_max + 1, c_min:c_max + 1] = 1

        for obs in self.dynamic_obstacles:
            oc = int(obs.x // GRID_SIZE)
            orow = int(obs.y // GRID_SIZE)
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    nr, nc = orow + dr, oc + dc
                    if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS:
                        self.inflated_grid[nr, nc] = 1

        sr, sc = self.start_cell
        gr, gc = self.goal_cell
        self.inflated_grid[sr, sc] = 0
        self.inflated_grid[gr, gc] = 0

    def step_dynamic_obstacles(self):
        for obs in self.dynamic_obstacles:
            obs.step(self.static_grid)
        self.update_inflated_grid()
