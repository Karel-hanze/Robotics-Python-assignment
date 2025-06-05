# Robotics-Python-assignment van Daan en Karel
Robotics + Python Assignment: Webots Simulation
This repository contains our solution for Lab 7 – Hardware-in-the-Loop Simulation. We've focused on implementing the robot control and pathfinding directly within Webots, meeting the core requirements for the assignment.

Table of Contents
Features
Getting Started
Dependencies
Running the Simulation
Demonstration Guide
Features
Path Planning: Implemented [Dijkstra's / A* / D* Lite] algorithm to find the shortest path for the robot to a predefined goal.
Obstacle Avoidance: The robot uses its proximity sensors to detect obstacles and re-plans its path automatically.
Path Visualization: The robot's planned path and current position are visualized during the simulation.
Getting Started
Dependencies
Python 3.x
Webots R2023b or later: The simulation environment.
Python Libraries:
pip install numpy
pip install matplotlib (for visualization)
Running the Simulation
Clone the Repository:
Bash

git clone https://github.com/your-username/your-repository-name.git
cd your-repository-name
Open in Webots:
Navigate to the webots_project_folder/ (or equivalent) and open the .wbt file (e.g., project.wbt) in Webots.
Start Simulation:
Once Webots loads the world, simply run the simulation directly within Webots. The robot's controller (written in Python) will handle the pathfinding and movement.
Demonstration Guide
For your demonstration, be prepared to show the following:

Path Planning: Start the simulation and demonstrate the robot successfully navigating from its starting position to a pre-defined goal using the shortest path.
Obstacle Avoidance: While the simulation is running, introduce a new obstacle (e.g., by moving an object in Webots) and show the robot detecting it and re-planning its path to reach the goal.
Visualization: Highlight the on-screen visualization of the map and the robot's planned path as it moves.
