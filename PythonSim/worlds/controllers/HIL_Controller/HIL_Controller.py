"""line_following_with_HIL controller."""
from controller import Robot
import numpy as np
import time
import matplotlib.pyplot as plt

# -------------------------------------------------------
# GRAPH en dijkstra functie toegevoegd

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

def dijkstra(graph, start, goal):
    """
    Bereken kortste pad van start naar goal in graph.
    graph is een dict van dicts: graph[node][neighbor] = gewicht.
    Retourneert: (afstand, lijst_van_knooppunten_in_pad)
    """
    unvisited = set(graph.keys())
    afstand = {node: float('inf') for node in graph}
    afstand[start] = 0
    parent = {node: None for node in graph}

    while unvisited:
        huidige = min(unvisited, key=lambda node: afstand[node])
        if afstand[huidige] == float('inf'):
            break
        unvisited.remove(huidige)

        if huidige == goal:
            break

        for buur, gewicht in graph[huidige].items():
            new_dist = afstand[huidige] + gewicht
            if new_dist < afstand[buur]:
                afstand[buur] = new_dist
                parent[buur] = huidige

    pad = []
    node = goal
    if parent[node] is not None or node == start:
        while node is not None:
            pad.insert(0, node)
            node = parent[node]
    return afstand[goal], pad

def plot_route(graph_coords, route_coords):
    # Plot alle knooppunten van het parcours
    x_all = [coord[0] for coord in graph_coords.values()]
    y_all = [coord[1] for coord in graph_coords.values()]
    plt.scatter(x_all, y_all, c='blue', label='Knooppunten')

    # Label de knooppunten
    for label, (x, y) in graph_coords.items():
        plt.text(x, y, label)

    # Plot de route als rode lijn
    if route_coords:
        x_route = [coord[0] for coord in route_coords]
        y_route = [coord[1] for coord in route_coords]
        plt.plot(x_route, y_route, 'r-', linewidth=2, label='Route')

    plt.title('Route van robot')
    plt.xlabel('X positie (m)')
    plt.ylabel('Y positie (m)')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()
    
# -------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

robot = Robot()
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = timestep/1000.0    # [s]

states = ['forward', 'turn_right', 'turn_left', 'stop']
current_state = 'stop'

# -------------------------------------------------------
# Initialize devices

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

# Initial robot pose
x = 0.195
y = -0.250
phi = 1.5708

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

# --------- Bereken het kortste pad automatisch ---------

SOURCE_NODE = 'B3'
DEST_NODE   = 'B2'

afst, path_labels = dijkstra(GRAPH, SOURCE_NODE, DEST_NODE)

if not path_labels:
    print(f"Geen pad gevonden van {SOURCE_NODE} naar {DEST_NODE}")
    path_labels = []  # fallback lege route
else:
    print(f"Kortste pad van {SOURCE_NODE} naar {DEST_NODE}: {path_labels} (afstand: {afst})")

route_coords = [COORDINATES[label] for label in path_labels]

current_wp_index = 0
DIST_THRESHOLD = 0.005
ANGLE_THRESHOLD = 0.02

current_state = 'stop'

while robot.step(timestep) != -1:

    gsValues = [gs[i].getValue() for i in range(3)]
    psValues = [ps[i].getValue() for i in range(8)]
    encoderValues = [encoder[i].getValue() for i in range(2)]
        
    if len(oldEncoderValues) < 2:
        oldEncoderValues = encoderValues.copy()

    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)

    if route_coords and current_wp_index < len(route_coords):
        target_x, target_y = route_coords[current_wp_index]
        dist = distance_to_point(x, y, target_x, target_y)
        ang_diff = angle_diff_to_point(x, y, phi, target_x, target_y)

        if dist < DIST_THRESHOLD:
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

    if current_state == 'forward':
        leftSpeed = speed
        rightSpeed = speed
    elif current_state == 'turn_right':
        leftSpeed = 0.5 * speed
        rightSpeed = -0.5 * speed
    elif current_state == 'turn_left':
        leftSpeed = -0.5 * speed
        rightSpeed = 0.5 * speed
    elif current_state == 'stop':
        leftSpeed = 0.0
        rightSpeed = 0.0

        
 

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    oldEncoderValues = encoderValues.copy()

    
    if route_coords and current_wp_index < len(route_coords):
        print(f"Pose: x={x:.3f} y={y:.3f} phi={phi:.3f} rad, State: {current_state}")
        print(f"Waypoint {current_wp_index+1}/{len(route_coords)} Target: {route_coords[current_wp_index]}")
    elif route_coords and current_wp_index >= len(route_coords):
        print(f"Arrived at desired waypoint")
        print(f"Pose: x={x:.3f} y={y:.3f} phi={phi:.3f} rad, State: Finished")
        
        # Plot de route
        plot_route(COORDINATES, route_coords)
        
        time.sleep(5)
        break
