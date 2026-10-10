"""
src/config.py
Central configuration for the AI-Based Autonomous Navigation System.
All kinematic, sensor, and grid parameters are defined here for reproducibility.
"""

# Window & Grid Resolution
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
SIM_VIEW_WIDTH = 840   # Left 840px for 2D environment canvas
HUD_PANEL_WIDTH = 360  # Right 360px for real-time telemetry HUD
GRID_SIZE = 20         # Each grid cell is 20x20 pixels
GRID_COLS = SIM_VIEW_WIDTH // GRID_SIZE  # 42 columns
GRID_ROWS = WINDOW_HEIGHT // GRID_SIZE   # 40 rows
FPS = 60

# Robot / Autonomous Vehicle Kinematics
ROBOT_RADIUS = 14.0           # Physical radius in pixels
SAFETY_MARGIN_CELLS = 1       # Obstacle inflation radius in grid cells
MAX_LINEAR_SPEED = 2.8        # Pixels per frame
MAX_ANGULAR_SPEED = 0.08      # Radians per frame
LOOKAHEAD_DISTANCE = 36.0     # Pure pursuit lookahead distance (pixels)
GOAL_TOLERANCE = 16.0         # Distance threshold to consider goal reached

# Synthetic LiDAR / Computer Vision Perception Parameters
LIDAR_NUM_RAYS = 36           # 360-degree scan (10-degree resolution)
LIDAR_MAX_RANGE = 180.0       # Maximum raycast sensing distance in pixels
EMERGENCY_STOP_DIST = 28.0    # Immediate reactive brake / replan distance

# Dynamic Obstacle Parameters
NUM_DYNAMIC_OBSTACLES = 4
DYNAMIC_OBSTACLE_SPEED = 1.15

# Color Palette (RGB)
COLOR_BG = (11, 14, 23)
COLOR_GRID = (26, 34, 52)
COLOR_WALL = (71, 85, 105)
COLOR_INFLATED = (30, 41, 59)
COLOR_EXPLORED = (14, 116, 144)
COLOR_PATH = (6, 182, 212)
COLOR_TRAIL = (16, 185, 129)
COLOR_ROBOT = (56, 189, 248)
COLOR_GOAL = (16, 185, 129)
COLOR_START = (99, 102, 241)
COLOR_DYNAMIC_OBS = (245, 158, 11)
COLOR_BBOX = (239, 68, 68)
