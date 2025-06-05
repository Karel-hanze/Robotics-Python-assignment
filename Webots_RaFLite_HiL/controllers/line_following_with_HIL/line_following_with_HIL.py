"""
Line Following Robot Controller with Waypoint Navigation and Ground Sensor Line Following.

This controller guides the robot along a predefined path of waypoints.
When the robot detects a line with its ground sensors, it engages in
proportional line-following. If no line is detected, it uses
waypoint navigation to steer towards the next target.

Tuning of LINE_DETECTION_THRESHOLD and KP_LINE_FOLLOW is critical.
"""

from controller import Robot
import numpy as np
import math

# --- Robot Constants ---
MAX_SPEED = 6.28  # Maximum motor speed in rad/s (typical for e-puck robot)
speed = 0.4 * MAX_SPEED  # Base forward speed (e.g., 40% of max speed)

R = 0.020  # Wheel radius in meters (typical for e-puck)
D = 0.057  # Distance between wheels (track width) in meters (typical for e-puck)

# --- Navigation Parameters ---
# Distance threshold to consider a waypoint 'reached' (in meters)
DIST_THRESHOLD = 0.005
# Angular threshold for waypoint navigation (in radians) before attempting a turn
ANGLE_THRESHOLD = 0.05

# --- Line Following Parameters (***THESE REQUIRE TUNING!***) ---
# Value that a ground sensor must exceed to be considered 'on the line'.
# You MUST verify your sensor readings to set this correctly.
# If sensor values are 0-100 on white, 800-1023 on black, 600.0 is reasonable.
# If sensor values are inverted (low on line, high off line), adjust this and line_error calculation.
LINE_DETECTION_THRESHOLD = 600.0

# Proportional gain for line following.
# Higher value = more aggressive correction. Too high = oscillation. Too low = poor tracking.
KP_LINE_FOLLOW = 0.008

# --- Robot Initialization ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())  # Simulation timestep in milliseconds
delta_t = timestep / 1000.0  # Convert timestep to seconds

# --- Device Initialization ---

# Proximity Sensors (ps): Enabled but not used in this specific logic
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# Encoders (wheel sensors): Used for odometry (pose estimation)
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
oldEncoderValues = []  # Stores previous encoder values for speed calculation

# Ground Sensors (gs): CRUCIAL for line following (assuming left, center, right)
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))  # Set motors to velocity control mode
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)  # Initial motor velocities
rightMotor.setVelocity(0.0)

# --- Initial Robot Pose ---
# ***IMPORTANT: Verify these values match your robot's actual starting position
# in your Webots world. Odometry errors accumulate from this point.***
x = 0.195    # Initial X coordinate in meters
y = -0.250   # Initial Y coordinate in meters
phi = 1.5708 # Initial orientation in radians (1.5708 rad = 90 degrees, pointing along positive Y-axis)

# --- Waypoint Definitions ---
# Dictionary of predefined coordinates for various points in your environment
COORDINATES = {
    'B1': (0.19, -0.35), 'B2': (0.295, -0.35), 'B3': (0.395, -0.35), 'B4': (0.5, -0.35),
    'M1': (0, -0.25), 'M2': (-0.5, -0.25), 'M3': (-0.5, -0.1), 'M4': (0, -0.1),
    'M5': (0.5, 0), 'M6': (0, 0), 'M7': (-0.5, 0), 'M8': (0, 0.1),
    'M9': (0.5, 0.1), 'M10': (0.5, 0.25), 'M11': (0, 0.25),
    'K1': (0.19, -0.25), 'K2': (0.295, -0.25), 'K3': (0.395, -0.25), 'K4': (0.5, -0.25),
    'K5': (-0.5, 0.25), 'K6': (-0.395, 0.25), 'K7': (-0.295, 0.25), 'K8': (-0.195, 0.25),
    'E1': (-0.5, 0.35), 'E2': (-0.395, 0.35), 'E3': (-0.295, 0.35), 'E4': (-0.195, 0.35),
}

# Define the robot's path as a list of coordinate labels
path_labels = ['B1', 'K1', 'M1', 'M4', 'M6', 'M8', 'M11', 'K8', 'K7', 'K6', 'K5', 'E1']
# Convert path labels to actual (x, y) coordinates
route_coords = [COORDINATES[label] for label in path_labels]
current_wp_index = 0 # Index of the current target waypoint in `route_coords`

# --- Helper Functions for Odometry and Navigation ---

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Calculates the angular velocity (rad/s) of the left and right wheels."""
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t  # Left wheel
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t  # Right wheel
    return wl, wr

def get_robot_speeds(wl, wr, r, d):
    """Calculates the robot's linear (u) and angular (w) speed."""
    u = r / 2.0 * (wr + wl)  # Linear speed (forward/backward)
    w = r / d * (wr - wl)    # Angular speed (rotation)
    return u, w

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Estimates the robot's new pose (x, y, phi) using differential drive odometry."""
    delta_phi = w * delta_t  # Change in orientation
    phi = phi_old + delta_phi
    
    # Normalize phi to be within [-pi, pi)
    phi = (phi + np.pi) % (2 * np.pi) - np.pi
    
    # Calculate change in x and y based on linear speed and new orientation
    delta_x = u * np.cos(phi) * delta_t
    delta_y = u * np.sin(phi) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi

def angle_diff_to_point(x, y, phi, target_x, target_y):
    """Calculates the angular difference (rad) from current orientation to target point."""
    desired_angle = math.atan2(target_y - y, target_x - x)
    
    diff = desired_angle - phi
    
    # Normalize the angle difference to be within [-pi, pi)
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return diff

def distance_to_point(x, y, target_x, target_y):
    """Calculates the Euclidean distance to a target point."""
    return math.sqrt((x - target_x)**2 + (y - target_y)**2)


# --- Main Control Loop ---
# This loop runs continuously as long as the Webots simulation is active
while robot.step(timestep) != -1:

    # 1. Read Sensor Values
    gsValues = [gs[i].getValue() for i in range(3)]  # Ground sensor readings
    psValues = [ps[i].getValue() for i in range(8)]  # Proximity sensor readings
    encoderValues = [encoder[i].getValue() for i in range(2)] # Encoder readings
        
    # Initialize oldEncoderValues on the first iteration
    if not oldEncoderValues: # Check if list is empty
        oldEncoderValues = encoderValues.copy()

    # 2. Update Robot Pose (Odometry)
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)

    # 3. Determine Control Mode: Line Following or Waypoint Navigation
    on_line = False
    # Check if any ground sensor detects a line based on the threshold
    if (gsValues[0] > LINE_DETECTION_THRESHOLD or
        gsValues[1] > LINE_DETECTION_THRESHOLD or
        gsValues[2] > LINE_DETECTION_THRESHOLD):
        on_line = True

    current_left_speed = 0.0
    current_right_speed = 0.0

    if on_line:
        # --- MODE: LINE FOLLOWING ---
        # Calculate line following error: (Left Sensor Value - Right Sensor Value)
        # Assuming higher values mean "on line".
        # If left sensor is higher (robot drifted right), error is positive -> turn left.
        # If right sensor is higher (robot drifted left), error is negative -> turn right.
        line_error = gsValues[0] - gsValues[2]
        
        # If your sensors are inverted (low values on line, high off line),
        # you might need to change this to: line_error = gsValues[2] - gsValues[0]
        # or use: line_error = (LINE_DETECTION_THRESHOLD - gsValues[0]) - (LINE_DETECTION_THRESHOLD - gsValues[2])

        # Apply proportional control for angular correction
        angular_correction = KP_LINE_FOLLOW * line_error
        
        # Limit the angular correction to prevent excessively sharp turns
        max_correction_magnitude = speed * 0.8 # Max allowed speed difference from base speed
        angular_correction = max(-max_correction_magnitude, min(max_correction_magnitude, angular_correction))

        # Adjust wheel speeds for line following:
        # Positive correction makes left wheel slower, right wheel faster (turn left)
        # Negative correction makes left wheel faster, right wheel slower (turn right)
        current_left_speed = speed - angular_correction
        current_right_speed = speed + angular_correction

        # Advance waypoint if reached while line following (important for turns/intersections)
        if route_coords and current_wp_index < len(route_coords):
            target_x, target_y = route_coords[current_wp_index]
            dist = distance_to_point(x, y, target_x, target_y)
            if dist < DIST_THRESHOLD:
                current_wp_index += 1
                print(f"DEBUG: Waypoint {current_wp_index}/{len(route_coords)} reached while line following!")
        
        # Debugging output for Line Following
        print(f"MODE: LINE_FOLLOW | GS: [{gsValues[0]:.0f}, {gsValues[1]:.0f}, {gsValues[2]:.0f}] Error={line_error:.0f} Corr={angular_correction:.3f}")

    else:
        # --- MODE: WAYPOINT NAVIGATION (No line detected or line lost) ---
        if route_coords and current_wp_index < len(route_coords):
            target_x, target_y = route_coords[current_wp_index]
            dist = distance_to_point(x, y, target_x, target_y)
            ang_diff = angle_diff_to_point(x, y, phi, target_x, target_y)

            if dist < DIST_THRESHOLD:
                # Current waypoint reached, move to the next one
                current_wp_index += 1
                if current_wp_index >= len(route_coords):
                    current_state = 'stop' # All waypoints processed
                else:
                    current_state = 'forward' # Transition to next waypoint
            else:
                # Not yet at the waypoint, determine steering based on angle difference
                if abs(ang_diff) > ANGLE_THRESHOLD:
                    # Angle is off, prioritize turning to align with waypoint
                    if ang_diff > 0: # Target is to the left
                        current_state = 'turn_left'
                    else: # Target is to the right
                        current_state = 'turn_right'
                else:
                    # Angle is aligned enough, move straight forward towards the waypoint
                    current_state = 'forward'
        else:
            current_state = 'stop' # No waypoints defined or all processed

        # Set speeds based on the determined waypoint navigation state
        if current_state == 'forward':
            current_left_speed = speed
            current_right_speed = speed
        elif current_state == 'turn_right':
            current_left_speed = 0.3 * speed # Slower turn speed to prevent overshooting
            current_right_speed = -0.3 * speed
        elif current_state == 'turn_left':
            current_left_speed = -0.3 * speed
            current_right_speed = 0.3 * speed
        else: # 'stop' state
            current_left_speed = 0.0
            current_right_speed = 0.0
        
        # Debugging output for Waypoint Navigation
        print(f"MODE: WAYPOINT_NAV | State: {current_state} Dist={dist:.3f} AngleDiff={ang_diff:.3f}")

    # 4. Apply Motor Velocities
    # Ensure calculated speeds do not exceed the robot's MAX_SPEED capacity
    max_current_speed = max(abs(current_left_speed), abs(current_right_speed))
    if max_current_speed > MAX_SPEED:
        current_left_speed = (current_left_speed / max_current_speed) * MAX_SPEED
        current_right_speed = (current_right_speed / max_current_speed) * MAX_SPEED

    leftMotor.setVelocity(current_left_speed)
    rightMotor.setVelocity(current_right_speed)

    # 5. Prepare for Next Iteration: Store current encoder values
    oldEncoderValues = encoderValues.copy()

    # 6. General Debugging Output
    print(f"Pose: x={x:.3f} y={y:.3f} phi={phi:.3f} rad")
    if route_coords and current_wp_index < len(route_coords):
        print(f"Target Waypoint {current_wp_index+1}/{len(route_coords)}: {route_coords[current_wp_index]}")