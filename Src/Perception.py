"""
src/perception.py
Implements 360-Degree Synthetic 2D LiDAR Raycasting and
OpenCV Computer Vision Obstacle Contour & Bounding Box Detection.
"""

import math
import cv2
import numpy as np
from dataclasses import dataclass
from src.config import (
    LIDAR_NUM_RAYS, LIDAR_MAX_RANGE, GRID_SIZE, GRID_ROWS, GRID_COLS,
    SIM_VIEW_WIDTH, WINDOW_HEIGHT
)


@dataclass
class LidarHit:
    angle: float
    distance: float
    hit_x: float
    hit_y: float
    detected: bool


@dataclass
class DetectedObject:
    bbox: tuple[int, int, int, int]
    centroid: tuple[int, int]
    distance_px: float
    label: str


class PerceptionModule:
    def __init__(self):
        self.lidar_scan: list[LidarHit] = []
        self.detected_objects: list[DetectedObject] = []
        self.min_obstacle_dist: float = LIDAR_MAX_RANGE

    def scan_lidar(self, robot_x: float, robot_y: float, robot_theta: float, env) -> list[LidarHit]:
        """Performs 360-degree raycasting from the robot pose."""
        self.lidar_scan.clear()
        min_dist = LIDAR_MAX_RANGE

        for i in range(LIDAR_NUM_RAYS):
            ray_angle = robot_theta + (2.0 * math.pi * i / LIDAR_NUM_RAYS)
            cos_a = math.cos(ray_angle)
            sin_a = math.sin(ray_angle)

            hit_found = False
            dist = 4.0
            hit_x = robot_x + cos_a * LIDAR_MAX_RANGE
            hit_y = robot_y + sin_a * LIDAR_MAX_RANGE

            while dist < LIDAR_MAX_RANGE:
                px = robot_x + cos_a * dist
                py = robot_y + sin_a * dist
                col = int(px // GRID_SIZE)
                row = int(py // GRID_SIZE)

                if row < 0 or row >= GRID_ROWS or col < 0 or col >= GRID_COLS or env.static_grid[row, col] == 1:
                    hit_found = True
                    hit_x, hit_y = px, py
                    break

                for obs in env.dynamic_obstacles:
                    if math.hypot(px - obs.x, py - obs.y) <= obs.radius:
                        hit_found = True
                        hit_x, hit_y = px, py
                        break

                if hit_found:
                    break
                dist += 4.0

            if hit_found and dist < min_dist:
                min_dist = dist

            self.lidar_scan.append(LidarHit(ray_angle, dist, hit_x, hit_y, hit_found))

        self.min_obstacle_dist = min_dist
        return self.lidar_scan

    def detect_obstacles_opencv(self, robot_x: float, robot_y: float, env) -> list[DetectedObject]:
        """Uses OpenCV thresholding and contour extraction to detect moving obstacles."""
        frame = np.zeros((WINDOW_HEIGHT, SIM_VIEW_WIDTH), dtype=np.uint8)
        for obs in env.dynamic_obstacles:
            cv2.circle(frame, (int(obs.x), int(obs.y)), int(obs.radius + 2), 255, -1)

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.detected_objects.clear()
        for idx, cnt in enumerate(contours):
            if cv2.contourArea(cnt) < 30:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2
            dist = math.hypot(cx - robot_x, cy - robot_y)
            if dist <= LIDAR_MAX_RANGE * 1.4:
                self.detected_objects.append(
                    DetectedObject(
                        bbox=(x, y, w, h),
                        centroid=(cx, cy),
                        distance_px=round(dist, 1),
                        label=f"DYN_OBJ_{idx + 1}"
                    )
                )
        return self.detected_objects
