import sys
import pygame
from simulation.world import SimulationWorld
from src.perception import PerceptionModule
from src.planner import AStarPlanner
from src.controller import NavigationController

def main():
    print("Initializing AI-Based Autonomous Navigation System...")
    
    # Initialize simulation world
    sim = SimulationWorld()
    perception = PerceptionModule(sim.grid)
    planner = AStarPlanner(sim.grid)
    controller = NavigationController()

    # Compute initial path
    print(f"Computing path from Start {sim.robot_pos} to Goal {sim.goal_pos}...")
    path = planner.plan(sim.robot_pos, sim.goal_pos)
    
    if not path:
        print("Error: No valid path found between start and goal!")
        sys.exit(1)
        
    print(f"Path successfully generated with {len(path)} waypoints.")
    running = True

    while running:
        # Event handling (allow user to close window)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Perception step
        detected_obstacles = perception.scan_environment(sim.robot_pos)
        
        # Navigation control step
        if path:
            next_pos = controller.get_next_action(sim.robot_pos, path)
            sim.robot_pos = next_pos
            path.pop(0)  # Remove reached waypoint

            # Check if goal is reached
            if sim.robot_pos == sim.goal_pos:
                print("Goal reached successfully!")
                sim.update_display(path)
                pygame.time.wait(2000)
                running = False

        # Render current state
        sim.update_display(path)

    pygame.quit()
    print("Simulation session terminated cleanly.")

if __name__ == "__main__":
    main()
