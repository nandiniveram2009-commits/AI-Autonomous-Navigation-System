"""
src/controller.py
Implements the Autonomous Vehicle / Mobile Robot kinematic state,
Pure Pursuit path tracker, and reactive obstacle avoidance.
"""

import math
from src.config import (
    GRID_SIZE, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED,
    LOOKAHEAD_DISTANCE, GOAL_TOLERANCE, EMERGENCY_STOP_DIST
)


class RobotAgent:
    def __init__(self, start_row: int, start_col: int):
        self.x = (start_col + 0.5) * GRID_SIZE
        self.y = (start_row + 0.5) * GRID_SIZE
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.cross_track_error = 0.0
        self.status = "IDLE"
        self.trajectory_history: list[tuple[float, float]] = [(self.x, self.y)]
        self.replan_count = 0

    def reset(self, start_row: int, start_col: int):
        self.x = (start_col + 0.5) * GRID_SIZE
        self.y = (start_row + 0.5) * GRID_SIZE
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.cross_track_error = 0.0
        self.status = "NAVIGATING"
        self.trajectory_history = [(self.x, self.y)]

    def current_cell(self) -> tuple[int, int]:
        return int(self.y // GRID_SIZE), int(self.x // GRID_SIZE)

    def needs_replan(self, path: list[tuple[int, int]], inflated_grid) -> bool:
        if not path:
            return True
        for r, c in path[:10]:
            if inflated_grid[r, c] == 1:
                self.replan_count += 1
                return True
        return False

    def follow_path(self, path: list[tuple[int, int]], goal_cell: tuple[int, int], lidar_scan) -> bool:
        goal_x = (goal_cell[1] + 0.5) * GRID_SIZE
        goal_y = (goal_cell[0] + 0.5) * GRID_SIZE
        dist_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)

        if dist_to_goal <= GOAL_TOLERANCE:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.status = "GOAL_REACHED"
            return True

        if not path:
            self.linear_vel = 0.0
            self.status = "WAITING_FOR_PATH"
            return False

        target_x, target_y = goal_x, goal_y
        min_cell_dist = float("inf")
        for r, c in path:
            wx = (c + 0.5) * GRID_SIZE
            wy = (r + 0.5) * GRID_SIZE
            d = math.hypot(wx - self.x, wy - self.y)
            if d < min_cell_dist:
                min_cell_dist = d
            if d >= LOOKAHEAD_DISTANCE:
                target_x, target_y = wx, wy
                break

        self.cross_track_error = round(min_cell_dist / GRID_SIZE, 3)
        desired_theta = math.atan2(target_y - self.y, target_x - self.x)

        repulsive_turn = 0.0
        min_front_dist = 999.0
        for hit in lidar_scan:
            if not hit.detected:
                continue
            rel_angle = math.atan2(math.sin(hit.angle - self.theta), math.cos(hit.angle - self.theta))
            if abs(rel_angle) < math.pi / 2.5 and hit.distance < EMERGENCY_STOP_DIST * 1.8:
                min_front_dist = min(min_front_dist, hit.distance)
                weight = (EMERGENCY_STOP_DIST * 1.8 - hit.distance) / (EMERGENCY_STOP_DIST * 1.8)
                repulsive_turn += -0.35 * weight if rel_angle >= 0 else 0.35 * weight

        heading_error = math.atan2(
            math.sin(desired_theta - self.theta),
            math.cos(desired_theta - self.theta)
        ) + repulsive_turn

        self.angular_vel = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 0.22 * heading_error))
        turn_slowdown = max(0.25, 1.0 - abs(heading_error) / math.pi)
        prox_slowdown = 0.35 if min_front_dist < EMERGENCY_STOP_DIST else 1.0
        self.linear_vel = MAX_LINEAR_SPEED * turn_slowdown * prox_slowdown
        self.status = "AVOIDING_OBSTACLE" if prox_slowdown < 1.0 else "NAVIGATING"

        self.theta += self.angular_vel
        self.x += self.linear_vel * math.cos(self.theta)
        self.y += self.linear_vel * math.sin(self.theta)
        self.trajectory_history.append((self.x, self.y))
        return False
