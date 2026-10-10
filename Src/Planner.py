"""
src/planner.py
Implements industry-standard graph search algorithms (A*, Dijkstra, BFS)
over an 8-connected occupancy grid with Octile distance heuristic.
"""

import heapq
from collections import deque
import numpy as np


class PathPlanner:
    NEIGHBORS_8 = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)
    ]

    @staticmethod
    def octile_heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (1.4142 - 2.0) * min(dx, dy)

    @classmethod
    def plan(
        cls,
        grid: np.ndarray,
        start: tuple[int, int],
        goal: tuple[int, int],
        algorithm: str = "astar"
    ) -> tuple[list[tuple[int, int]], set[tuple[int, int]]]:
        rows, cols = grid.shape
        if start == goal:
            return [start], {start}

        if algorithm.lower() == "bfs":
            return cls._bfs(grid, start, goal)

        open_heap = []
        heapq.heappush(open_heap, (0.0, 0.0, start))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        explored: set[tuple[int, int]] = set()

        while open_heap:
            _, current_g, current = heapq.heappop(open_heap)
            if current in explored:
                continue
            explored.add(current)

            if current == goal:
                return cls._reconstruct_path(came_from, current), explored

            for dr, dc, move_cost in cls.NEIGHBORS_8:
                nr, nc = current[0] + dr, current[1] + dc
                neighbor = (nr, nc)

                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if grid[nr, nc] == 1 and neighbor != goal:
                    continue

                # Prevent diagonal corner-cutting
                if dr != 0 and dc != 0:
                    if grid[current[0] + dr, current[1]] == 1 or grid[current[0], current[1] + dc] == 1:
                        continue

                tentative_g = current_g + move_cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = cls.octile_heuristic(neighbor, goal) if algorithm.lower() == "astar" else 0.0
                    f = tentative_g + h
                    heapq.heappush(open_heap, (f, tentative_g, neighbor))

        return [], explored

    @classmethod
    def _bfs(cls, grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]):
        rows, cols = grid.shape
        queue = deque([start])
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        explored: set[tuple[int, int]] = {start}

        while queue:
            current = queue.popleft()
            if current == goal:
                return cls._reconstruct_path(came_from, current), explored
            for dr, dc, _ in cls.NEIGHBORS_8[:4]:
                nr, nc = current[0] + dr, current[1] + dc
                neighbor = (nr, nc)
                if 0 <= nr < rows and 0 <= nc < cols and neighbor not in explored:
                    if grid[nr, nc] == 0 or neighbor == goal:
                        explored.add(neighbor)
                        came_from[neighbor] = current
                        queue.append(neighbor)
        return [], explored

    @staticmethod
    def _reconstruct_path(came_from: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
