
#AI based autonomous navigation system 
## Overview 

Modern autonomous systems—from warehouse mobile robots (AMRs) to self-driving cars—rely on robust software architectures to safely navigate unstructured environments. This project implements a complete, transparent autonomous navigation pipeline from scratch in Python. It demonstrates how an autonomous agent perceives its surroundings, computes optimal collision-free trajectories, and executes waypoint navigation in real-time.
## Problem statement 

Manual navigation in dynamic, hazardous, or repetitive environments is prone to human error, latency, and operational risk. Developing physical robotic prototypes to test navigation algorithms is costly and logistically challenging for students and researchers.
This project solves these challenges by providing a virtual, physics-based simulation environment that models perception, planning, and control loops locally on any standard laptop, allowing rapid prototyping and algorithmic validation without requiring dedicated hardware.
## Industry relevance 

* **Autonomous Vehicles (AVs):** Waymo and Tesla utilize perception-to-control loops for street-level urban navigation.
* **Warehouse Automation (AMRs):** Companies like Amazon Robotics deploy grid-based graph search and reactive collision avoidance for inventory logistics.
* **Last-Mile Delivery:** Sidewalk delivery robots rely on real-time path corrections to navigate around pedestrians and dynamic obstacles safely.
* **Industrial Safety:** Automated guided vehicles (AGVs) in manufacturing plants use structured costmaps to transport materials reli
## Tech stack 

**Language:** Python 3.10+
* **Simulation & Rendering:** Pygame 2.5.2
* **Mathematical Operations & Matrices:** NumPy 1.26.4
* **Visualization & Plotting:** Matplotlib 3.8.3
* **Path Planning Algorithm:** $A^*$ (A-Star) Graph Search with Heuristic Cost Estimation
## Architecture 

[Simulation Environment ]
                │
                ▼
[ Perception Module ] ──> Scans local occupancy matrix for obstacles
                │
               ▼
[ A* Path Planner ] ──> Computes optimal waypoint trajectory
                │
               ▼
[ Control Module ] ──> Translates waypoints into agent steering actions
                │
                ▼
[ Agent Execution ] ──> Updates simulation canvas state at 15 FPS
## Folder structure 

---

## 📁 Folder Structure
```text
AI-Autonomous-Navigation-System 
│
├── src/
│   ├── __init__.py
│   ├── perception.py
│   ├── planner.py
│   ├──environment.py
│   ├──config.py │    
│   └── controller.py
│
├── README.md
├── requirements.txt
├── .gitignore
└── main.py
## Simulation workflow 

Environment Initialization: SimulationWorld generates a grid-based spatial map containing static walls, obstacles, a start node, and a destination goal.

Environmental Sensing: PerceptionModule scans the agent's immediate 8-connected neighborhood to detect obstacles.

Trajectory Planning: AStarPlanner evaluates open and closed sets using a heuristic cost function to establish the shortest collision-free coordinate sequence.

Execution & Control: NavigationController drives the agent step-by-step along the planned waypoints until the target state is successfully reached
## Results 

Successfully computes optimal paths across complex obstacle layouts in real-time (<10ms
 execution latency).
Demonstrates robust collision avoidance and waypoint tracking in a closed-loop simulation loop running smoothly at 15 FPS
## Future improvement 

ROS2 Integration: Migrate Python modules into ROS2 nodes (perception_node, planner_node) communicating over standard topics.

CARLA Simulator Integration: Upgrade from the 2D Pygame grid to a high-fidelity 3D physics simulator for autonomous vehicles.

Deep Reinforcement Learning (DRL): Implement PPO (Proximal Policy Optimization) using PyTorch to train end-to-end neural network driving policies.

SLAM Implementation: Incorporate Simultaneous Localization and Mapping to allow the robot to map unknown environments dynamically
## Learning outcomes 

Gained deep practical understanding of robotic software architectures separating perception, planning, and control.
Mastered graph search algorithms (A) and heuristic distance metrics for optimal route finding.
Built professional project structuring habits suitable for open-source contributions and portfolio showcases.
## Author

* **GitHub:** [nandiniveram2009](https://github.com)
* **LinkedIn:** [Nandini Verma](https://linkedin.com)



## Future improvement 

ROS2 Integration: Migrate Python modules into ROS2 nodes (perception_node, planner_node) communicating over standard topics.

CARLA Simulator Integration: Upgrade from the 2D Pygame grid to a high-fidelity 3D physics simulator for autonomous vehicles.

Deep Reinforcement Learning (DRL): Implement PPO (Proximal Policy Optimization) using PyTorch to train end-to-end neural network driving policies.

SLAM Implementation: Incorporate Simultaneous Localization and Mapping to allow the robot to map unknown environments dynamically