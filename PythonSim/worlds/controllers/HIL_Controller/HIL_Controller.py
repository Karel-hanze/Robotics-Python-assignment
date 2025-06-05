from controller import Robot
import numpy as np
import time
import matplotlib.pyplot as plt

# --- Graph and Dijkstra's Algorithm ---

# Define the graph representing the robot's environment.
# Each key is a node, and its value is a dictionary of connected neighbors
# with their respective edge weights (distances).
GRAPH = {
    'B1': {'K1': 0.1}, 'B2': {'K2': 0.1}, 'B3': {'K3': 0.1}, 'B4': {'K4': 0.1},
    'M1': {'K1': 0.195, 'M2': 0.5, 'M4': 0.15},
    'M2': {'M1': 0.5, 'M3': 0.15},
    'M3': {'M2': 0.15, 'M4': 0.5, 'M7': 0.1},
    'M4': {'M1': 0.15, 'M3': 0.5, 'M6': 0.1},
    'M5': {'K4': 0.25, 'M6': 0.5, 'M9': 0.1},
    'M6': {'M4': 0.1, 'M5': 0.5, 'M7': 0.5, 'M8': 0.1},
    'M7': {'K5': 0.25, 'M3': 0.1, 'M6': 0.5},
    'M8': {'M6': 0.1, 'M9': 0.5, 'M11': 0.15},
    'M9': {'M5': 0.1, 'M8': 0.5, 'M10': 0.15},
    'M10': {'M9': 0.15, 'M11': 0.5},
    'M11': {'K8': 0.195, 'M8': 0.15, 'M10': 0.5},
    'K1': {'K2': 0.1, 'M1': 0.195, 'B1': 0.1},
    'K2': {'K1': 0.1, 'K3': 0.1, 'B2': 0.1},
    'K3': {'K2': 0.1, 'K4': 0.1, 'B3': 0.1},
    'K4': {'K3': 0.1, 'M5': 0.25, 'B4': 0.1},
    'K5': {'K6': 0.1, 'M7': 0.25, 'E1': 0.1},
    'K6': {'K5': 0.1, 'K7': 0.1, 'E2': 0.1},
    'K7': {'K6': 0.1, 'K8': 0.1, 'E3': 0.1},
    'K8': {'K7': 0.1, 'M11': 0.195, 'E4': 0.1},
    'E1': {'K5': 0.1}, 'E2': {'K6': 0.1}, 'E3': {'K7': 0.1}, 'E4': {'K8': 0.1}
}


def dijkstra(graph, start, goal, blocked_paths=None):
    """
    Calculates the shortest path from a start node to a goal node in a graph.

    Args:
        graph (dict): A dictionary representing the graph, where keys are nodes
                      and values are dictionaries of neighbors with edge weights.
        start (str): The label of the starting node.
        goal (str): The label of the goal node.
        blocked_paths (list, optional): A list of tuples, where each tuple
                                        (node1, node2) represents a path segment
                                        that cannot be traversed. Defaults to None.

    Returns:
        tuple: A tuple containing:
                - float: The total distance of the shortest path.
                - list: A list of node labels representing the shortest path.
                        Returns an empty list if no path is found.
    """
    if blocked_paths is None:
        blocked_paths = []

    unvisited = set(graph.keys())
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    parents = {node: None for node in graph}

    while unvisited:
        # Find the unvisited node with the smallest distance
        current_node = min(unvisited, key=lambda node: distances[node])

        # If the smallest distance is infinity, no more reachable nodes
        if distances[current_node] == float('inf'):
            break

        unvisited.remove(current_node)

        # If goal is reached, break
        if current_node == goal:
            break

        for neighbor, weight in graph[current_node].items():
            # Check if the path segment is blocked in either direction
            if (current_node, neighbor) in blocked_paths or \
                    (neighbor, current_node) in blocked_paths:
                continue

            new_dist = distances[current_node] + weight
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                parents[neighbor] = current_node

    path = []
    node = goal
    # Reconstruct the path if the goal was reachable
    if distances[goal] != float('inf'):
        while node is not None:
            path.insert(0, node)
            node = parents[node]
    return distances[goal], path


# --- Robot Initialization and Configuration ---

MAX_SPEED = 6.28  # Maximum speed of the robot's wheels
DEFAULT_SPEED_FACTOR = 0.4  # Factor to apply to MAX_SPEED for general movement
robot = Robot()
timestep = int(robot.getBasicTimeStep())  # Simulation timestep in milliseconds
delta_t = timestep / 1000.0  # Simulation timestep in seconds

# Define possible robot states for state machine control
ROBOT_STATES = ['forward', 'turn_right', 'turn_left', 'stop', 'obstacle_avoidance']
current_robot_state = 'stop'

# Initialize proximity sensors (for obstacle detection)
proximity_sensors = []
proximity_sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps_sensor = robot.getDevice(proximity_sensor_names[i])
    ps_sensor.enable(timestep)
    proximity_sensors.append(ps_sensor)

# Initialize wheel encoders (for odometry)
wheel_encoders = []
encoder_names = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder = robot.getDevice(encoder_names[i])
    encoder.enable(timestep)
    wheel_encoders.append(encoder)
previous_encoder_values = []  # Stores encoder values from the previous step

# Initialize ground sensors (for line following, though not actively used in this version)
ground_sensors = []
ground_sensor_names = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs_sensor = robot.getDevice(ground_sensor_names[i])
    gs_sensor.enable(timestep)
    ground_sensors.append(gs_sensor)

# Initialize wheel motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))  # Set to infinity for velocity control
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Robot physical parameters
ROBOT_WHEEL_RADIUS = 0.020  # meters
ROBOT_AXLE_LENGTH = 0.057  # meters (distance between wheels)

# Define precise coordinates for each node in the graph for plotting and navigation
COORDINATES = {
    'B1': (0.19, -0.35), 'B2': (0.295, -0.35), 'B3': (0.395, -0.35), 'B4': (0.5, -0.35),
    'M1': (0, -0.25), 'M2': (-0.5, -0.25), 'M3': (-0.5, -0.1), 'M4': (0, -0.1),
    'M5': (0.5, 0), 'M6': (0, 0), 'M7': (-0.5, 0), 'M8': (0, 0.1),
    'M9': (0.5, 0.1), 'M10': (0.5, 0.25), 'M11': (0, 0.25),
    'K1': (0.19, -0.25), 'K2': (0.295, -0.25), 'K3': (0.395, -0.25), 'K4': (0.5, -0.25),
    'K5': (-0.5, 0.25), 'K6': (-0.395, 0.25), 'K7': (-0.295, 0.25), 'K8': (-0.195, 0.25),
    'E1': (-0.5, 0.35), 'E2': (-0.395, 0.35), 'E3': (-0.295, 0.35), 'E4': (-0.195, 0.35),
}


# --- Robot Kinematics Functions ---

def get_wheel_speeds(current_encoder_values, previous_encoder_values, dt):
    """Calculate the angular speeds of the left and right wheels."""
    left_wheel_angular_speed = (current_encoder_values[0] - previous_encoder_values[0]) / dt
    right_wheel_angular_speed = (current_encoder_values[1] - previous_encoder_values[1]) / dt
    return left_wheel_angular_speed, right_wheel_angular_speed


def get_robot_velocities(left_angular_speed, right_angular_speed, wheel_radius, axle_length):
    """
    Calculate the robot's linear and angular velocities from wheel speeds.

    Args:
        left_angular_speed (float): Angular speed of the left wheel (rad/s).
        right_angular_speed (float): Angular speed of the right wheel (rad/s).
        wheel_radius (float): Radius of the robot's wheels (m).
        axle_length (float): Distance between the robot's wheels (m).

    Returns:
        tuple: A tuple containing:
                - float: Robot's linear velocity (m/s).
                - float: Robot's angular velocity (rad/s).
    """
    linear_velocity = wheel_radius / 2.0 * (right_angular_speed + left_angular_speed)
    angular_velocity = wheel_radius / axle_length * (right_angular_speed - left_angular_speed)
    return linear_velocity, angular_velocity


def update_robot_pose(linear_velocity, angular_velocity, x_old, y_old, phi_old, dt):
    """
    Update the robot's pose (x, y, orientation) using differential drive kinematics.

    Args:
        linear_velocity (float): Robot's linear velocity (m/s).
        angular_velocity (float): Robot's angular velocity (rad/s).
        x_old (float): Previous X coordinate.
        y_old (float): Previous Y coordinate.
        phi_old (float): Previous orientation (radians).
        dt (float): Time step (seconds).

    Returns:
        tuple: A tuple containing:
                - float: New X coordinate.
                - float: New Y coordinate.
                - float: New orientation (radians, normalized to [-pi, pi)).
    """
    delta_phi = angular_velocity * dt
    phi = phi_old + delta_phi

    # Normalize angle to [-pi, pi)
    phi = (phi + np.pi) % (2 * np.pi) - np.pi

    delta_x = linear_velocity * np.cos(phi) * dt
    delta_y = linear_velocity * np.sin(phi) * dt
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi


def angle_to_point(current_x, current_y, current_phi, target_x, target_y):
    """
    Calculates the angular difference from the robot's current orientation
    to the direction of a target point.

    Args:
        current_x (float): Robot's current X coordinate.
        current_y (float): Robot's current Y coordinate.
        current_phi (float): Robot's current orientation (radians).
        target_x (float): Target point's X coordinate.
        target_y (float): Target point's Y coordinate.

    Returns:
        float: The angular difference (radians) to turn to face the target.
               Positive for left turn, negative for right turn.
    """
    desired_angle = np.arctan2(target_y - current_y, target_x - current_x)
    diff = desired_angle - current_phi
    # Normalize angle difference to [-pi, pi)
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return diff


def calculate_distance(x1, y1, x2, y2):
    """Calculates the Euclidean distance between two points."""
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# --- Plotting Setup ---

# Determine min/max coordinates for stable plot limits based on all defined nodes
min_x = min(coord[0] for coord in COORDINATES.values())
max_x = max(coord[0] for coord in COORDINATES.values())
min_y = min(coord[1] for coord in COORDINATES.values())
max_y = max(coord[1] for coord in COORDINATES.values())

# Add a small buffer to the plot limits for better visualization
PLOT_BUFFER = 0.1
plot_xlim = (min_x - PLOT_BUFFER, max_x + PLOT_BUFFER)
plot_ylim = (min_y - PLOT_BUFFER, max_y + PLOT_BUFFER)

# Set up the Matplotlib plot once at the beginning
# Create the figure and a single subplot (axes)
fig, ax = plt.subplots(figsize=(9, 9))  # Set a fixed figure size (width, height in inches)
plt.ion()  # Turn on interactive mode for non-blocking updates


def plot_navigation_status(graph_coordinates, initial_path_coords, current_route_coords, blocked_segments,
                           current_robot_pose=None):
    """
    Plots the robot's navigation status, including nodes, paths, blocked segments,
    and the robot's current position and orientation.

    Args:
        graph_coordinates (dict): Dictionary of all node labels and their (x,y) coordinates.
        initial_path_coords (list): List of (x,y) coordinates representing the initial planned route.
        current_route_coords (list): List of (x,y) coordinates representing the currently active route.
        blocked_segments (list): List of tuples ((node1_label, node2_label)) of blocked paths.
        current_robot_pose (tuple, optional): Current robot pose (x, y, phi). Defaults to None.
    """
    ax.clear()  # Clear the specific axes for fresh plotting

    # Set fixed axis limits to keep the map stable
    ax.set_xlim(plot_xlim)
    ax.set_ylim(plot_ylim)

    # Plot all nodes on the course
    x_all_nodes = [coord[0] for coord in graph_coordinates.values()]
    y_all_nodes = [coord[1] for coord in graph_coordinates.values()]
    ax.scatter(x_all_nodes, y_all_nodes, c='blue', s=50, zorder=5, label='Nodes')

    # Label the nodes for clarity
    for label, (x, y) in graph_coordinates.items():
        ax.text(x + 0.01, y + 0.01, label, fontsize=9)

    # --- Add lines between connected nodes from the GRAPH dictionary ---
    for node, neighbors in GRAPH.items():
        x1, y1 = graph_coordinates[node]
        for neighbor in neighbors:
            # Avoid drawing duplicate lines (e.g., A-B and B-A)
            if (neighbor, node) in blocked_segments:  # Check if this specific direction is blocked
                continue  # Skip if already drawn as blocked
            if node in GRAPH.get(neighbor, {}):  # Check if the reverse connection exists to avoid drawing twice
                if node > neighbor:  # Arbitrary order to draw each line only once
                    continue

            x2, y2 = graph_coordinates[neighbor]
            # Draw a light grey dashed line for possible routes
            ax.plot([x1, x2], [y1, y2], 'darkgrey', linestyle='--', linewidth=2, zorder=1)
    # --- End of added lines for graph routes ---

    # Plot the initial planned path in green
    if initial_path_coords:
        x_initial = [coord[0] for coord in initial_path_coords]
        y_initial = [coord[1] for coord in initial_path_coords]
        ax.plot(x_initial, y_initial, 'g--', linewidth=2, alpha=0.7, label='Initial Route')

    # Plot the current active route in red
    if current_route_coords:
        x_route = [coord[0] for coord in current_route_coords]
        y_route = [coord[1] for coord in current_route_coords]
        # Only plot in red if it's different from the initial path (i.e., a re-route)
        if current_route_coords != initial_path_coords:
            ax.plot(x_route, y_route, 'r-', linewidth=2, label='Current Route (Re-routed)')
        else:
            # If the current route is still the initial one, plot it as solid green for emphasis
            ax.plot(x_route, y_route, 'g-', linewidth=2, label='Current Route (Initial)')

    # Plot blocked path segments as crosses
    for node1_label, node2_label in blocked_segments:
        if node1_label in graph_coordinates and node2_label in graph_coordinates:
            x1, y1 = graph_coordinates[node1_label]
            x2, y2 = graph_coordinates[node2_label]
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            # Plot a large 'x' at the midpoint of the blocked segment
            ax.plot(mid_x, mid_y, 'kx', markersize=12, markeredgewidth=3, zorder=10)
            # Draw a faint dashed line for the blocked path
            ax.plot([x1, x2], [y1, y2], 'k:', linewidth=1.5, alpha=0.6)

    # Plot the current robot position and orientation
    if current_robot_pose:
        robot_x, robot_y, robot_phi = current_robot_pose
        ax.plot(robot_x, robot_y, 'o', color='black', markersize=10, zorder=15, label='Robot Position')
        # Draw an arrow to indicate robot orientation
        arrow_length = 0.07
        ax.arrow(robot_x, robot_y, arrow_length * np.cos(robot_phi), arrow_length * np.sin(robot_phi),
                 head_width=0.03, head_length=0.04, fc='black', ec='black', zorder=15)

    ax.set_title('Robot Navigation Status', fontsize=14)
    ax.set_xlabel('X position (m)', fontsize=12)
    ax.set_ylabel('Y position (m)', fontsize=12)
    ax.legend(loc='upper right')
    ax.set_aspect('equal', adjustable='box')  # Keep aspect ratio equal and adjust box
    ax.grid(True)
    fig.canvas.draw_idle()  # Update plot without blocking for interactive figures
    fig.canvas.flush_events()  # Process events
    plt.pause(0.000001)  # Smallest possible pause to allow GUI events to process


# --- Path Planning and Navigation Control ---

# Define key nodes for the robot's mission
SPAWN_NODE = 'K1'  # Initial spawn point of the robot
START_MISSION_NODE = 'B2'  # First significant waypoint to reach for the main mission
DESTINATION_NODE = 'E4'  # Final destination for the main mission

# List to store dynamically blocked path segments
blocked_paths = []

# Nodes where obstacle detection should be temporarily ignored (e.g., at start/end zones)
IGNORE_OBSTACLE_NODES = ['E1', 'E2', 'E3', 'E4', 'B1', 'B2', 'B3', 'B4']
IGNORE_ZONE_RADIUS = 0.1  # Radius around ignore nodes to define the "ignore zone"

# --- Initial Path Calculation ---
initial_path_labels = []

# Calculate the initial path from the robot's spawn to the mission start node
_, path_to_start = dijkstra(GRAPH, SPAWN_NODE, START_MISSION_NODE)

# Calculate the initial path from the mission start node to the final destination
_, path_to_destination = dijkstra(GRAPH, START_MISSION_NODE, DESTINATION_NODE)

# Combine both paths to form the complete initial route (avoiding duplicate middle node)
if path_to_start and path_to_destination:
    # Use START_MISSION_NODE as the common point to merge
    initial_path_labels = path_to_start + path_to_destination[1:]
    print(f"Initial planned route: {initial_path_labels}")
else:
    print("Error: No complete initial path found for the mission.")
    initial_path_labels = []  # Fallback to an empty route

# Convert node labels to coordinates for plotting
if not initial_path_labels:
    initial_path_coordinates = []
    current_route_coordinates = []
else:
    initial_path_coordinates = [COORDINATES[label] for label in initial_path_labels]
    current_route_coordinates = initial_path_coordinates[:]  # Current active route starts as the initial one

# Set initial robot pose based on the first node of the planned route
if initial_path_labels:
    robot_x, robot_y = COORDINATES[initial_path_labels[0]]
else:
    # Fallback to a default starting point if no path is found
    robot_x = 0.195
    robot_y = -0.250
robot_orientation = np.pi / 2  # Initial orientation (e.g., facing positive Y)

current_waypoint_index = 0
DISTANCE_THRESHOLD = 0.005  # Distance to target waypoint to consider it reached
ANGLE_THRESHOLD = 0.02  # Angular difference to target direction to start moving forward
OBSTACLE_DETECTION_THRESHOLD = 100  # Proximity sensor reading threshold for obstacle detection

current_robot_state = 'stop'  # Initial state of the robot

# Initialize last_known_node to the first node of the path to enable re-planning from a valid point
last_known_node = initial_path_labels[0] if initial_path_labels else None

# --- Simulation Loop Control ---
PLOT_AND_PRINT_INTERVAL = 20  # Update plot and print messages every N simulation steps
step_counter = 0  # Counter for simulation steps

# State variables to track the robot's mission progress
mission_to_destination_completed = False  # True when robot reaches DESTINATION_NODE
returning_to_spawn = False  # True when robot is on its way back to SPAWN_NODE

# --- Main Simulation Loop ---
while robot.step(timestep) != -1:
    step_counter += 1

    # Read sensor values
    ground_sensor_values = [gs_sensor.getValue() for gs_sensor in ground_sensors]
    proximity_sensor_values = [ps_sensor.getValue() for ps_sensor in proximity_sensors]
    current_encoder_values = [encoder.getValue() for encoder in wheel_encoders]

    # Initialize previous_encoder_values on the first step
    if not previous_encoder_values:
        previous_encoder_values = current_encoder_values.copy()

    # Update robot pose (odometry)
    left_wheel_speed_rad_s, right_wheel_speed_rad_s = get_wheel_speeds(current_encoder_values, previous_encoder_values,
                                                                       delta_t)
    robot_linear_velocity, robot_angular_velocity = get_robot_velocities(left_wheel_speed_rad_s,
                                                                         right_wheel_speed_rad_s, ROBOT_WHEEL_RADIUS,
                                                                         ROBOT_AXLE_LENGTH)
    robot_x, robot_y, robot_orientation = update_robot_pose(robot_linear_velocity, robot_angular_velocity, robot_x,
                                                            robot_y, robot_orientation, delta_t)

    # Check if the robot is currently within an "ignore obstacle" zone
    in_ignore_zone = False
    for node_label in IGNORE_OBSTACLE_NODES:
        node_x, node_y = COORDINATES[node_label]
        if calculate_distance(robot_x, robot_y, node_x, node_y) < IGNORE_ZONE_RADIUS:
            in_ignore_zone = True
            break

    # Obstacle detection using front proximity sensors (ps0 and ps7)
    front_obstacle_detected = proximity_sensor_values[0] > OBSTACLE_DETECTION_THRESHOLD or \
                              proximity_sensor_values[7] > OBSTACLE_DETECTION_THRESHOLD

    # --- Obstacle Avoidance Logic ---
    if front_obstacle_detected and not in_ignore_zone and current_robot_state != 'obstacle_avoidance':
        if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
            print("Obstacle detected! Halting and recalculating path.")
        current_robot_state = 'obstacle_avoidance'
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

        # Mark the current segment as blocked if robot was moving towards a waypoint.
        # The segment to block is from 'last_known_node' to the 'current_target_waypoint'.

        current_target_waypoint_coord = None
        current_target_waypoint_label = None

        if current_route_coordinates and current_waypoint_index < len(current_route_coordinates):
            current_target_waypoint_coord = current_route_coordinates[current_waypoint_index]
            # Find the node label corresponding to the current target waypoint's coordinates
            for label, coord in COORDINATES.items():
                if coord == current_target_waypoint_coord:
                    current_target_waypoint_label = label
                    break

        if last_known_node and current_target_waypoint_label:
            blocked_node_1 = last_known_node
            blocked_node_2 = current_target_waypoint_label

            # Add to blocked_paths, checking both directions for robustness
            if (blocked_node_1, blocked_node_2) not in blocked_paths and \
                    (blocked_node_2, blocked_node_1) not in blocked_paths:
                blocked_paths.append((blocked_node_1, blocked_node_2))
                if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                    print(f"Blocked path segment: {blocked_node_1} <-> {blocked_node_2}")
        else:
            if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                print("Could not determine segment to block (last_known_node or target waypoint missing).")

        # Recalculate path from the last known good node to the overall destination
        if last_known_node and last_known_node in GRAPH:
            recalc_start_node = last_known_node

            # Determine the correct overall destination for recalculation
            overall_mission_destination = SPAWN_NODE if returning_to_spawn else DESTINATION_NODE

            distance_to_dest, new_path_labels = dijkstra(GRAPH, recalc_start_node, overall_mission_destination,
                                                         blocked_paths)

            if not new_path_labels or new_path_labels[0] != recalc_start_node:
                if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                    print("No new path found or path doesn't start from the correct node. Robot is stuck.")
                current_robot_state = 'stop'  # Remain stopped if no new path
                current_route_coordinates = []
            else:
                if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                    print(
                        f"New path found from {recalc_start_node}: {new_path_labels} (Distance: {distance_to_dest:.2f} m)")
                current_route_coordinates = [COORDINATES[label] for label in new_path_labels]
                current_waypoint_index = 0  # Reset waypoint index for the new path
                current_robot_state = 'forward'  # Resume forward movement after successful recalculation
        else:
            if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                print("Robot unable to determine a valid starting point for path recalculation. Stuck.")
            current_robot_state = 'stop'
            current_route_coordinates = []

    # Resume navigation if obstacle clears or robot enters ignore zone
    elif current_robot_state == 'obstacle_avoidance' and (not front_obstacle_detected or in_ignore_zone):
        if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
            print(f"Obstacle {'cleared' if not front_obstacle_detected else 'ignored (in zone)'}, resuming navigation.")
        current_robot_state = 'forward'

    # --- Normal Navigation Logic (Waypoint Following) ---
    if current_robot_state != 'obstacle_avoidance':
        if current_route_coordinates and current_waypoint_index < len(current_route_coordinates):
            target_x, target_y = current_route_coordinates[current_waypoint_index]
            dist_to_target = calculate_distance(robot_x, robot_y, target_x, target_y)
            angle_to_target = angle_to_point(robot_x, robot_y, robot_orientation, target_x, target_y)

            if dist_to_target < DISTANCE_THRESHOLD:
                # Waypoint reached, move to the next
                # Update last_known_node only when a waypoint is successfully reached
                if current_waypoint_index < len(current_route_coordinates):
                    # Get the label of the recently reached waypoint
                    reached_waypoint_coord = current_route_coordinates[current_waypoint_index]
                    for label, coord in COORDINATES.items():
                        if coord == reached_waypoint_coord:
                            last_known_node = label
                            break
                current_waypoint_index += 1

                # Check if the current mission segment (to DESTINATION_NODE or SPAWN_NODE) is complete
                if current_waypoint_index >= len(current_route_coordinates):
                    if not mission_to_destination_completed:
                        # Robot has reached DESTINATION_NODE, now plan path back to SPAWN_NODE
                        mission_to_destination_completed = True
                        returning_to_spawn = True
                        current_robot_state = 'stop'  # Temporarily stop to plan return

                        if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                            print(
                                f"Reached DESTINATION_NODE: {DESTINATION_NODE}. Planning return to SPAWN_NODE: {SPAWN_NODE}")

                        # Recalculate path from DESTINATION_NODE back to SPAWN_NODE
                        _, new_path_labels = dijkstra(GRAPH, DESTINATION_NODE, SPAWN_NODE, blocked_paths)
                        if new_path_labels:
                            current_route_coordinates = [COORDINATES[label] for label in new_path_labels]
                            current_waypoint_index = 0
                            # Update last_known_node for the start of the return path
                            if new_path_labels:
                                last_known_node = new_path_labels[0]
                            current_robot_state = 'forward'
                            if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                                print(f"Return path planned: {new_path_labels}")
                        else:
                            if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                                print("Could not find a return path to spawn. Robot stuck.")
                            current_robot_state = 'stop'

                    elif returning_to_spawn:
                        # Robot has successfully returned to SPAWN_NODE
                        current_robot_state = 'stop'
                        if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                            print(f"Successfully returned to SPAWN_NODE: {SPAWN_NODE}. Mission complete.")

                        # **Crucially, stop the motors here.**
                        left_motor.setVelocity(0.0)
                        right_motor.setVelocity(0.0)

                        # Plot the final route one last time before exiting
                        plot_navigation_status(COORDINATES, initial_path_coordinates, current_route_coordinates,
                                               blocked_paths, (robot_x, robot_y, robot_orientation))
                        time.sleep(5)  # Pause simulation for final view
                        break  # Exit the main loop as mission is fully complete
                else:
                    current_robot_state = 'forward'  # Continue moving to the next waypoint
            else:
                # Determine turn or go forward based on angle to target
                if abs(angle_to_target) > ANGLE_THRESHOLD:
                    if angle_to_target > 0:
                        current_robot_state = 'turn_left'
                    else:
                        current_robot_state = 'turn_right'
                else:
                    current_robot_state = 'forward'  # Head towards target if alignment is good
        else:
            current_robot_state = 'stop'  # No path or all waypoints reached

    # Set motor speeds based on the determined state
    left_motor_speed = 0.0
    right_motor_speed = 0.0

    if current_robot_state == 'forward':
        left_motor_speed = DEFAULT_SPEED_FACTOR * MAX_SPEED
        right_motor_speed = DEFAULT_SPEED_FACTOR * MAX_SPEED
    elif current_robot_state == 'turn_right':
        left_motor_speed = 0.5 * DEFAULT_SPEED_FACTOR * MAX_SPEED
        right_motor_speed = -0.5 * DEFAULT_SPEED_FACTOR * MAX_SPEED
    elif current_robot_state == 'turn_left':
        left_motor_speed = -0.5 * DEFAULT_SPEED_FACTOR * MAX_SPEED
        right_motor_speed = 0.5 * DEFAULT_SPEED_FACTOR * MAX_SPEED
    elif current_robot_state in ['stop', 'obstacle_avoidance']:
        left_motor_speed = 0.0
        right_motor_speed = 0.0

    left_motor.setVelocity(left_motor_speed)
    right_motor.setVelocity(right_motor_speed)

    # Store current encoder values for the next step's odometry calculation
    previous_encoder_values = current_encoder_values.copy()

    # --- Conditional Plotting and Printing ---
    if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
        current_robot_pose_for_plot = (robot_x, robot_y, robot_orientation)
        plot_navigation_status(COORDINATES, initial_path_coordinates, current_route_coordinates, blocked_paths,
                               current_robot_pose_for_plot)

        # Print detailed status updates
        if current_route_coordinates and current_waypoint_index < len(
                current_route_coordinates) and current_robot_state != 'obstacle_avoidance':
            # Safely get target node label if path_labels exist and index is valid
            target_node_label = "N/A"
            if initial_path_labels and current_waypoint_index < len(initial_path_labels):
                target_node_label = initial_path_labels[current_waypoint_index]
            elif current_waypoint_index < len(current_route_coordinates):  # For re-routed paths
                target_waypoint_coord = current_route_coordinates[current_waypoint_index]
                for label, coord in COORDINATES.items():
                    if coord == target_waypoint_coord:
                        target_node_label = label
                        break

            print(
                f"Pose: X={robot_x:.3f} Y={robot_y:.3f} Phi={np.degrees(robot_orientation):.1f}° | State: {current_robot_state}")
            print(
                f"Target Waypoint: {target_node_label} ({current_waypoint_index + 1}/{len(current_route_coordinates)}) | Last Known Node: {last_known_node}")
        elif current_waypoint_index >= len(current_route_coordinates) and current_robot_state != 'obstacle_avoidance':
            if not mission_to_destination_completed:
                print(f"Arrived at DESTINATION_NODE: {DESTINATION_NODE}")
            elif returning_to_spawn:
                print(f"Arrived at SPAWN_NODE: {SPAWN_NODE}. Mission complete.")
                print(f"Final Pose: X={robot_x:.3f} Y={robot_y:.3f} Phi={np.degrees(robot_orientation):.1f}°")
        elif current_robot_state == 'obstacle_avoidance':
            print(
                f"Pose: X={robot_x:.3f} Y={robot_y:.3f} Phi={np.degrees(robot_orientation):.1f}° | State: {current_robot_state} (Obstacle! In Ignore Zone: {in_ignore_zone})")
            print(f"Last Known Node for Recalculation: {last_known_node}")