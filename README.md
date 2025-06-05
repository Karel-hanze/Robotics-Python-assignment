# Robotics-Python-assignment van Daan en Karel

Webots Robot Navigation
This repository contains the Python controller code for a robot navigating a predefined environment in Webots. The robot uses Dijkstra's algorithm for path planning, odometry for pose estimation, and proximity sensors for obstacle avoidance and dynamic re-routing.

Features
Graph-Based Navigation: The robot navigates a pre-defined graph representing its environment, with nodes and weighted edges.
Dijkstra's Algorithm: Implements Dijkstra's algorithm to find the shortest path between a start and goal node.
Dynamic Path Planning: The robot can detect obstacles and dynamically re-plan its path to avoid blocked segments, ensuring mission completion.
Odometry: Estimates the robot's pose (position and orientation) using wheel encoder data.
Waypoint Following: Controls the robot's motors to follow a series of waypoints determined by the path planning algorithm.
Real-time Visualization: Integrates with Matplotlib to provide a real-time plot of the robot's position, planned paths, current route, and blocked segments within the environment.
Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

Dependencies
This project requires the following:

Python 3.x
Webots R2023b (or a compatible version)
pip packages:
numpy
matplotlib
You can install the Python dependencies using pip:

Bash

pip install numpy matplotlib
Reproducing the Main Experiment
To reproduce the robot's navigation experiment in Webots:

Open Webots: Launch the Webots simulation software.
Load the World: Open the Webots world file associated with this controller. (e.g., your_robot_world.wbt). This README assumes you have a Webots world set up with a robot that matches the sensor and motor configurations in the provided Python code.
Place the Controller: In the Webots scene tree, locate your robot node. Under its controller field, select your_controller_filename.py (the Python file provided).
Run the Simulation: Press the "Run" button (green play icon) in Webots.
The robot will:

Calculate an initial path from its SPAWN_NODE ('K1') to START_MISSION_NODE ('B2'), and then from 'B2' to DESTINATION_NODE ('E4').
Begin navigating along this planned path.
Continuously monitor its front proximity sensors for obstacles.
If an obstacle is detected (and the robot is not in an IGNORE_OBSTACLE_NODES zone), it will stop, mark the segment as blocked, and recalculate a new path to its current overall mission destination (either DESTINATION_NODE or SPAWN_NODE if returning).
Once DESTINATION_NODE ('E4') is reached, the robot will plan a return path to SPAWN_NODE ('K1') and navigate back.
The Matplotlib plot will update in real-time, showing the robot's progress, the initial planned path, the current active path, and any segments that have been blocked due to obstacles.
Code Structure and Docstrings
The core logic of the robot's behavior is encapsulated in the your_controller_filename.py file. Each function within this file includes a docstring that explains its purpose, arguments, return values, and any side effects.

Key Functions:
dijkstra(graph, start, goal, blocked_paths=None): Implements Dijkstra's shortest path algorithm.
get_wheel_speeds(current_encoder_values, previous_encoder_values, dt): Calculates individual wheel angular speeds.
get_robot_velocities(left_angular_speed, right_angular_speed, wheel_radius, axle_length): Determines the robot's linear and angular velocities.
update_robot_pose(linear_velocity, angular_velocity, x_old, y_old, phi_old, dt): Updates the robot's estimated position and orientation using odometry.
angle_to_point(current_x, current_y, current_phi, target_x, target_y): Calculates the required turning angle to face a target point.
calculate_distance(x1, y1, x2, y2): Computes the Euclidean distance between two points.
plot_navigation_status(...): Visualizes the robot's environment, paths, and current state using Matplotlib.
