"""line_following_with_HIL controller."""
from controller import Robot
import numpy as np
import time
import matplotlib.pyplot as plt

# --- GRAPH and Dijkstra function ---

GRAPH = {
    'B1': {'K1': 0.1},
    'B2': {'K2': 0.1},
    'B3': {'K3': 0.1},
    'B4': {'K4': 0.1},
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
    'E1': {'K5': 0.1},
    'E2': {'K6': 0.1},
    'E3': {'K7': 0.1},
    'E4': {'K8': 0.1}
}

def dijkstra(graph, start, goal, blocked_paths=None):
    """
    Calculates the shortest path from start to goal in a graph.
    The graph is a dictionary of dictionaries: graph[node][neighbor] = weight.
    blocked_paths: A list of tuples (node1, node2) indicating blocked paths.
    Returns: (distance, list_of_nodes_in_path)
    """
    if blocked_paths is None:
        blocked_paths = []

    unvisited = set(graph.keys())
    distance = {node: float('inf') for node in graph}
    distance[start] = 0
    parent = {node: None for node in graph}

    while unvisited:
        current = min(unvisited, key=lambda node: distance[node])
        if distance[current] == float('inf'):
            break
        unvisited.remove(current)

        if current == goal:
            break

        for neighbor, weight in graph[current].items():
            # Check if the path is blocked in either direction
            if (current, neighbor) in blocked_paths or (neighbor, current) in blocked_paths:
                continue

            new_dist = distance[current] + weight
            if new_dist < distance[neighbor]:
                distance[neighbor] = new_dist
                parent[neighbor] = current

    path = []
    node = goal
    # Only build path if the goal was reachable
    if distance[goal] != float('inf'):
        while node is not None:
            path.insert(0, node)
            node = parent[node]
    return distance[goal], path


# --- Initialize variables (COORDINATES defined here for plot limits) ---

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

robot = Robot()
timestep = int(robot.getBasicTimeStep())    # [ms]
delta_t = timestep/1000.0    # [s]

states = ['forward', 'turn_right', 'turn_left', 'stop', 'obstacle_avoidance']
current_state = 'stop'

# --- Initialize devices ---

ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
oldEncoderValues = []

gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

COORDINATES = {
    'B1': (0.19, -0.35),
    'B2': (0.295, -0.35),
    'B3': (0.395, -0.35),
    'B4': (0.5, -0.35),
    'M1': (0, -0.25),
    'M2': (-0.5, -0.25),
    'M3': (-0.5, -0.1),
    'M4': (0, -0.1),
    'M5': (0.5, 0),
    'M6': (0, 0),
    'M7': (-0.5, 0),
    'M8': (0, 0.1),
    'M9': (0.5, 0.1),
    'M10': (0.5, 0.25),
    'M11': (0, 0.25),
    'K1': (0.19, -0.25),
    'K2': (0.295, -0.25),
    'K3': (0.395, -0.25),
    'K4': (0.5, -0.25),
    'K5': (-0.5, 0.25),
    'K6': (-0.395, 0.25),
    'K7': (-0.295, 0.25),
    'K8': (-0.195, 0.25),
    'E1': (-0.5, 0.35),
    'E2': (-0.395, 0.35),
    'E3': (-0.295, 0.35),
    'E4': (-0.195, 0.35),
}

R = 0.020
D = 0.057

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t
    return wl, wr

def get_robot_speeds(wl, wr, r, d):
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)
    return u, w

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    
    delta_x = u * np.cos(phi) * delta_t
    delta_y = u * np.sin(phi) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi

def angle_diff_to_point(x, y, phi, target_x, target_y):
    import math
    desired_angle = math.atan2(target_y - y, target_x - x)
    diff = desired_angle - phi
    while diff > math.pi:
        diff -= 2*math.pi
    while diff < -math.pi:
        diff += 2*math.pi
    return diff

def distance_to_point(x, y, target_x, target_y):
    return ((x - target_x)**2 + (y - target_y)**2)**0.5

# --- Plot Setup with fixed limits ---

# Determine min/max coordinates for stable plot limits
min_x = min(coord[0] for coord in COORDINATES.values())
max_x = max(coord[0] for coord in COORDINATES.values())
min_y = min(coord[1] for coord in COORDINATES.values())
max_y = max(coord[1] for coord in COORDINATES.values())

# Add a small buffer to the limits
buffer = 0.1
plot_xlim = (min_x - buffer, max_x + buffer)
plot_ylim = (min_y - buffer, max_y + buffer)

# Set up the plot once at the beginning
plt.figure(figsize=(8, 8)) # Optional: Set a fixed figure size (width, height in inches)
plt.ion() # Turn on interactive mode for non-blocking updates

def plot_route(graph_coords, initial_path_coords, current_route_coords, blocked_paths, current_robot_pose=None):
    plt.clf() # Clear the current figure for fresh plotting

    # Set fixed axis limits
    plt.xlim(plot_xlim)
    plt.ylim(plot_ylim)

    # Plot all nodes on the course
    x_all = [coord[0] for coord in graph_coords.values()]
    y_all = [coord[1] for coord in graph_coords.values()]
    plt.scatter(x_all, y_all, c='blue', label='Nodes')

    # Label the nodes
    for label, (x, y) in graph_coords.items():
        plt.text(x, y, label)

    # Plot the initial path in green
    if initial_path_coords:
        x_initial = [coord[0] for coord in initial_path_coords]
        y_initial = [coord[1] for coord in initial_path_coords]
        plt.plot(x_initial, y_initial, 'g-', linewidth=2, label='Initial Route')

    # Plot the current route (re-calculated or remaining) in red
    if current_route_coords:
        x_route = [coord[0] for coord in current_route_coords]
        y_route = [coord[1] for coord in current_route_coords]
        # Only plot in red if it's different from the initial path (i.e., a re-route)
        if current_route_coords != initial_path_coords:
            plt.plot(x_route, y_route, 'r-', linewidth=2, label='Current Route (Re-routed)')
        else:
            # If the current route is still the initial one, it's covered by the green line.
            pass # No need to plot again, as initial path is already green

    # Plot blocked paths as crosses
    for node1_label, node2_label in blocked_paths:
        if node1_label in graph_coords and node2_label in graph_coords:
            x1, y1 = graph_coords[node1_label]
            y1 = graph_coords[node1_label][1]
            x2, y2 = graph_coords[node2_label]
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            # Plot a large 'x' at the midpoint of the blocked segment
            plt.plot(mid_x, mid_y, 'kx', markersize=10, markeredgewidth=2, label='Blocked Path')
            # Optionally, draw a faint dashed line for the blocked path
            plt.plot([x1, x2], [y1, y2], 'k--', linewidth=1, alpha=0.5)

    # Plot the current robot position
    if current_robot_pose:
        rx, ry, rphi = current_robot_pose
        plt.plot(rx, ry, 'o', color='black', markersize=8, label='Robot Position')
        # Draw an arrow to indicate robot orientation
        arrow_length = 0.05
        plt.arrow(rx, ry, arrow_length * np.cos(rphi), arrow_length * np.sin(rphi),
                  head_width=0.02, head_length=0.03, fc='black', ec='black')

    plt.title('Robot Route and Obstacles')
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    plt.legend()
    plt.axis('equal') # Keep aspect ratio equal
    plt.grid(True)
    plt.draw() # Use plt.draw() to update plot without blocking
    plt.pause(0.000001) # Smallest possible pause to allow GUI events to process


# --- Path planning ---

# Define the full route as a sequence of nodes based on your combined path logic
Spawn_NODE = 'K1'      # Startpositie robot
Start_NODE = 'B2'      # Eerste tussenpunt (bijvoorbeeld kruising)
DEST_NODE  = 'E4'     # Eindpositie

# List to store blocked paths (tuples of (node1, node2))
blocked_paths = []

# Function to find the nearest node to the robot's current position (not directly used for re-planning but useful for debugging/understanding)
def find_nearest_node(robot_x, robot_y, coordinates_map):
    min_dist = float('inf')
    nearest_node = None
    for node, (nx, ny) in coordinates_map.items():
        dist = distance_to_point(robot_x, robot_y, nx, ny)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node

# Nodes where obstacles should be ignored
IGNORE_OBSTACLE_NODES = ['E1', 'E2', 'E3', 'E4', 'B1', 'B2', 'B3', 'B4']
IGNORE_ZONE_RADIUS = 0.1 # A small radius around the ignore nodes to define the zone

# --- Initial path calculation based on the full route ---
path_labels = []

# Calculate initial path from Spawn_NODE to Start_NODE
_, path1 = dijkstra(GRAPH, Spawn_NODE, Start_NODE)

# Calculate initial path from Start_NODE to DEST_NODE
_, path2 = dijkstra(GRAPH, Start_NODE, DEST_NODE)

# Combine both paths to one route (remove duplicate middle node)
if path1 and path2:
    path_labels = path1 + path2[1:]  # Avoid duplicate Start_NODE
    print(f"Initial planned route: {path_labels}")
else:
    print("No complete path found based on the defined FULL_ROUTE_NODES.")
    path_labels = [] # fallback empty route

if not path_labels:
    initial_path_coords = [] # fallback empty route
    route_coords = []
else:
    initial_path_coords = [COORDINATES[label] for label in path_labels]
    route_coords = initial_path_coords[:] # Current route starts as the initial one

# Initial robot pose: Set to the first node of the planned route
if path_labels:
    x, y = COORDINATES[path_labels[0]]
else:
    # Fallback if no path is found, or set to a default starting point
    x = 0.195
    y = -0.250
phi = 1.5708 # Initial orientation

current_wp_index = 0
DIST_THRESHOLD = 0.005
ANGLE_THRESHOLD = 0.02
OBSTACLE_DISTANCE_THRESHOLD = 100 # Adjust this based on your sensor range and robot speed

current_state = 'stop'
# Initialize last_known_node to the first node of the path, or SOURCE_NODE if you prefer
last_known_node = path_labels[0] if path_labels else None 

# --- NEW: Control plotting and printing frequency ---
PLOT_AND_PRINT_INTERVAL = 20 # Only update plot and print every 20 simulation steps
step_counter = 0

# --- Main simulation loop ---
while robot.step(timestep) != -1:
    step_counter += 1 # Increment counter every step

    # Read sensor values
    gsValues = [gs[i].getValue() for i in range(3)]
    psValues = [ps[i].getValue() for i in range(8)]
    encoderValues = [encoder[i].getValue() for i in range(2)]
        
    if len(oldEncoderValues) < 2:
        oldEncoderValues = encoderValues.copy()

    # Update robot pose
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)

    # Check if the robot is currently in an "ignore obstacle" zone
    in_ignore_zone = False
    for node_label in IGNORE_OBSTACLE_NODES:
        node_x, node_y = COORDINATES[node_label]
        if distance_to_point(x, y, node_x, node_y) < IGNORE_ZONE_RADIUS:
            in_ignore_zone = True
            break

    # Obstacle detection (front sensors ps0, ps7)
    front_obstacle_detected = psValues[0] > OBSTACLE_DISTANCE_THRESHOLD or psValues[7] > OBSTACLE_DISTANCE_THRESHOLD

    # Obstacle avoidance logic
    if front_obstacle_detected and not in_ignore_zone and current_state != 'obstacle_avoidance':
        if step_counter % PLOT_AND_PRINT_INTERVAL == 0: # Only print when updating visuals
            print("Obstacle detected! Going back to last node and recalculating path.")
        current_state = 'obstacle_avoidance'
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)

        # Mark the current segment as blocked
        if path_labels and current_wp_index < len(path_labels): 
            # The segment that was intended to be traversed is (last_known_node, next_waypoint_label)
            if current_wp_index > 0:
                blocked_node_1 = path_labels[current_wp_index - 1]
            else: # If we are at the very beginning of the path
                blocked_node_1 = path_labels[0] # Consider the first node as the origin of the current segment

            blocked_node_2 = path_labels[current_wp_index]
            
            # Add to blocked_paths, checking both directions for robustness
            if (blocked_node_1, blocked_node_2) not in blocked_paths and \
               (blocked_node_2, blocked_node_1) not in blocked_paths:
                blocked_paths.append((blocked_node_1, blocked_node_2))
            if step_counter % PLOT_AND_PRINT_INTERVAL == 0: # Only print when updating visuals
                print(f"Blocked path: {blocked_node_1} <-> {blocked_node_2}")

        # Recalculate path from the last known good node to the overall DEST_NODE.
        if last_known_node and last_known_node in GRAPH: # Ensure last_known_node is valid
            recalc_start_node = last_known_node
            overall_dest_node = DEST_NODE # The very last node in the desired route
            
            afst, path_labels = dijkstra(GRAPH, recalc_start_node, overall_dest_node, blocked_paths)
            
            if not path_labels or path_labels[0] != recalc_start_node:
                if step_counter % PLOT_AND_PRINT_INTERVAL == 0: # Only print when updating visuals
                    print("No new path found or path doesn't start from the correct node. Robot is stuck.")
                current_state = 'stop' # Remain stopped if no new path
                route_coords = []
            else:
                if step_counter % PLOT_AND_PRINT_INTERVAL == 0: # Only print when updating visuals
                    print(f"New path found from {recalc_start_node}: {path_labels} (distance: {afst})")
                route_coords = [COORDINATES[label] for label in path_labels]
                current_wp_index = 0 # Reset waypoint index for the new path
                current_state = 'forward' # Resume forward movement after recalculation
        else:
            if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
                print("Robot is unable to determine a starting point for path recalculation. Stuck.")
            current_state = 'stop'
            route_coords = []


    elif current_state == 'obstacle_avoidance' and (not front_obstacle_detected or in_ignore_zone):
        # If obstacle is gone OR we've entered an ignore zone, resume normal navigation
        if step_counter % PLOT_AND_PRINT_INTERVAL == 0: # Only print when updating visuals
            if in_ignore_zone:
                print("Entered ignore zone, resuming navigation.")
            elif not front_obstacle_detected:
                print("Obstacle cleared, resuming navigation.")
        current_state = 'forward'
        
    # Normal navigation logic (only if not in obstacle avoidance)
    if current_state != 'obstacle_avoidance':
        if route_coords and current_wp_index < len(route_coords):
            target_x, target_y = route_coords[current_wp_index]
            dist = distance_to_point(x, y, target_x, target_y)
            ang_diff = angle_diff_to_point(x, y, phi, target_x, target_y)

            if dist < DIST_THRESHOLD:
                # Update last_known_node when a waypoint is reached successfully
                if current_wp_index < len(path_labels): # Ensure index is valid for path_labels
                    last_known_node = path_labels[current_wp_index]
                current_wp_index += 1
                if current_wp_index >= len(route_coords):
                    current_state = 'stop'
                else:
                    current_state = 'forward'
            else:
                if abs(ang_diff) > ANGLE_THRESHOLD:
                    if ang_diff > 0:
                        current_state = 'turn_left'
                    else:
                        current_state = 'turn_right'
                else:
                    current_state = 'forward'
        else:
            current_state = 'stop'

    # Set motor speeds based on current state
    if current_state == 'forward':
        leftSpeed = speed
        rightSpeed = speed
    elif current_state == 'turn_right':
        leftSpeed = 0.5 * speed
        rightSpeed = -0.5 * speed
    elif current_state == 'turn_left':
        leftSpeed = -0.5 * speed
        rightSpeed = 0.5 * speed
    elif current_state == 'stop' or current_state == 'obstacle_avoidance':
        leftSpeed = 0.0
        rightSpeed = 0.0
        
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    oldEncoderValues = encoderValues.copy()

    # --- Plotting and Printing - now conditional ---
    if step_counter % PLOT_AND_PRINT_INTERVAL == 0:
        current_robot_pose = (x, y, phi)
        # Pass both initial and current route coordinates
        plot_route(COORDINATES, initial_path_coords, route_coords, blocked_paths, current_robot_pose)

        if route_coords and current_wp_index < len(route_coords) and current_state != 'obstacle_avoidance':
            print(f"Pose: x={x:.3f} y={y:.3f} phi={phi:.3f} rad, State: {current_state}")
            print(f"Current Target Waypoint: {path_labels[current_wp_index]} ({current_wp_index+1}/{len(route_coords)})")
            print(f"Last Known Node: {last_known_node}")
        elif route_coords and current_wp_index >= len(route_coords) and current_state != 'obstacle_avoidance':
            print(f"Arrived at desired waypoint")
            print(f"Pose: x={x:.3f} y={y:.3f} phi={phi:.3f} rad, State: Finished")
            
            # Plot the final route one last time
            plot_route(COORDINATES, initial_path_coords, route_coords, blocked_paths, current_robot_pose)
            
            time.sleep(5) # This will pause the simulation, which might not be desired for speed
            break
        elif current_state == 'obstacle_avoidance':
            print(f"Pose: x={x:.3f} y={y:.3f} phi={phi:.3f} rad, State: {current_state} - Obstacle in front! (Ignore Zone: {in_ignore_zone})")
            print(f"Last Known Node for Recalculation: {last_known_node}")

    # Break condition for completed path (moved outside conditional print block)
    if route_coords and current_wp_index >= len(route_coords) and current_state != 'obstacle_avoidance' and step_counter % PLOT_AND_PRINT_INTERVAL != 0:
        time.sleep(5) # Still pauses for 5 seconds on completion
        break