#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 07:43:50 2025

@author: fran
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Simulation parameters
dt = 0.1  # time step
T = 60.0  # total simulation time
steps = int(T / dt)


""" # UAV state vector: [x, y, z, vx, vy, vz, ax, ay, az]
state = np.zeros(9)
state[:3] = [0.0, 0.0, 0.0]  # initial position
state[3:6] = [2.0, 2.0, 0.0]  # vx = 1 m/s to the right """

# System dynamics matrix A (9x9)
A = np.eye(9)
for i in range(3):
    A[i, i+3] = dt           # position update: x += v * dt
    A[i, i+6] = 0.5 * dt**2  # position update: x += 0.5 * a * dt^2
    A[i+3, i+6] = dt         # velocity update: v += a * dt
    

# Control dynamics: only in the acceleration terms
B = np.zeros((9, 3))

# Position rows (x, y, z) ← from acceleration
B[0, 0] = 0.5 * dt**2
B[1, 1] = 0.5 * dt**2
B[2, 2] = 0.5 * dt**2

# Velocity rows (vx, vy, vz) ← from acceleration
B[3, 0] = dt
B[4, 1] = dt
B[5, 2] = dt

# Acceleration rows (ax, ay, az) ← set directly to control
B[6, 0] = 1.0
B[7, 1] = 1.0
B[8, 2] = 1.0
    
# Noise matrix W (9x1), noise only in acceleration
W = A
a_noise_std = 0.1


# Environment boundary rectangle and inner obstacles
xmin, xmax = -30.0, 30.0
ymin, ymax = -20.0, 20.0

map_bounds = (xmin, ymin, xmax, ymax)
# map_bounds = (-20, -15, 40, 30)

walls = []

# Sensor parameters
lidar_range = 12.0
lidar_resolution_deg = 1
num_rays = int(360 / lidar_resolution_deg) + 1
sensor_angles = np.radians(np.linspace(-180, 180, 361))
sensor_noise_cm = np.random.uniform(0.01, 0.03, size=num_rays)

# Threshold distances
warning_distance = 12.0
danger_distance = 2.0

mass = 1.5  # kg
v_max = 2
a_max = 10

max_thrust = 30  # Maximum thrust (needs tuning based on your drone's mass and desired acceleration)
base_thrust = mass * 9.81  # Approximate thrust to counteract gravity
max_pitch_roll = np.radians(30)  # Maximum allowed pitch and roll angles (e.g., +/- 20 degrees)
max_yaw_rate = np.radians(90)

# Define a constant for the pitch/roll scaling factor
PITCH_ROLL_SCALING_FACTOR = 0.5
YAW_RATE_SCALING_FACTOR = 0.5


def simulate_lidar_2d(state):
    lidar_data = []
    origin = state[:2]
    for i, angle in enumerate(sensor_angles):
        min_dist = lidar_range
        found_obstacle = False  # Flag to check if an obstacle was detected
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        for wall in get_rect_walls(map_bounds):
            intersect = ray_intersect(origin, ray_dir, wall[0], wall[1])
            if intersect is not None:
                dist = np.linalg.norm(intersect - origin)
                if dist < min_dist:
                    min_dist = dist
                    found_obstacle = True
        if found_obstacle:
            min_dist += sensor_noise_cm[i]  # add noise only if obstacle detected
            lidar_data.append((min_dist, angle))
        else:
            lidar_data.append((lidar_range, angle)) # Return max range if no obstacle
    return lidar_data

def get_rect_walls(bounds):
    x, y, xmax, ymax = bounds  # Unpack the bounds into x, y (bottom-left) and xmax, ymax (top-right)
    return [
        ((x, y), (xmax, y)),          # Bottom side
        ((xmax, y), (xmax, ymax)),    # Right side
        ((xmax, ymax), (x, ymax)),    # Top side
        ((x, ymax), (x, y))           # Left side
    ]


def ray_intersect(origin, direction, p1, p2):
    p = origin
    r = direction
    q = np.array(p1)
    s = np.array(p2) - q
    r_cross_s = np.cross(r, s)
    if r_cross_s == 0:
        return None
    t = np.cross((q - p), s) / r_cross_s
    u = np.cross((q - p), r) / r_cross_s
    if 0 <= u <= 1 and t >= 0:
        return p + t * r
    return None


def evasion_strategy(lidar_data, warning_distance, danger_distance, max_acceleration):
    """
    Evasion strategy based on lidar data to generate acceleration commands.

    Args:
        lidar_data (list of tuples): List of (distance, angle_rad) to obstacles.
        danger_distance (float): Distance threshold for strong avoidance.
        warning_distance (float): Distance threshold for weaker avoidance.
        max_acceleration (float): Maximum allowed acceleration magnitude.

    Returns:
        tuple: (ax, ay, az) - desired accelerations in x, y, and z.
    """
    distances = np.array([d for d, _ in lidar_data])
    angles_rad = np.array([a for _, a in lidar_data])

    if not distances.size:
        return 0.0, 0.0, 0.0  # No lidar data, no avoidance

    min_dist_index = np.argmin(distances)
    min_dist = distances[min_dist_index]
    min_angle_rad = angles_rad[min_dist_index]

    ax = 0.0
    ay = 0.0
    az = 0.0

    if min_dist < danger_distance:
        # Strong avoidance: Steer directly away from the closest obstacle
        avoidance_angle = min_angle_rad + np.pi
        strength = max_acceleration  # Use maximum acceleration for immediate danger
        ax = strength * np.cos(avoidance_angle)
        ay = strength * np.sin(avoidance_angle)
    elif min_dist < warning_distance:
        # Weaker avoidance: Steer away, with strength proportional to proximity
        avoidance_angle = min_angle_rad + np.pi
        # Scale strength linearly from 0 at warning_distance to max_acceleration at danger_distance
        strength = max_acceleration * (warning_distance - min_dist) / (warning_distance - danger_distance)
        ax = strength * np.cos(avoidance_angle)
        ay = strength * np.sin(avoidance_angle)

    return ax, ay, az


def process_model(state):
    noise = np.zeros(9)
    noise[6:] = np.random.normal(0, a_noise_std, 3)
    new_state = A @ state + W @ noise
    # Clip velocities to max speed
    new_state[3:6] = np.clip(new_state[3:6], -v_max, v_max)
    # Clip velocities to max speed
    new_state[6:9] = np.clip(new_state[6:9], -a_max, a_max)
    return new_state


def update_state(state, control):
    state[6:] = control # Pone ax, ay, az en el estado
    # state[6:] = np.clip(control, -a_max, a_max) # control
    return process_model(state)


def plot_scene(state, bounds, lidar_data):
    plt.clf()  # Clear the previous plot
    ax = plt.gca()

    # Unpack the bounds: xmin, ymin, xmax, ymax
    xmin, ymin, xmax, ymax = bounds

    # Set the limits of the plot slightly beyond the boundary for better view
    ax.set_xlim(xmin - 2, xmax + 2)
    ax.set_ylim(ymin - 2, ymax + 2)

    # Get the walls of the rectangle using the get_rect_walls function
    walls = get_rect_walls(bounds)

    # Plot the walls of the rectangular boundary
    for wall_start, wall_end in walls:
        ax.plot([wall_start[0], wall_end[0]], [wall_start[1], wall_end[1]], 'k-', lw=2)

    # Plot the drone's position
    x, y = state[0], state[1]  # State position: [x, y, z] from state vector
    plt.plot(x, y, 'bo', markersize=6)  # Drone as a blue dot

    # Plot the lidar measurements and obstacle info
    min_dist_obstacle = float('inf')
    min_angle_deg_obstacle = 0

    drone_x, drone_y = state[0], state[1]
    for d, a_rad in lidar_data:
        dx = d * np.cos(a_rad)  # x displacement based on lidar distance and angle
        dy = d * np.sin(a_rad)  # y displacement based on lidar distance and angle
        plt.plot([drone_x, drone_x + dx], [drone_y, drone_y + dy], 'r--', linewidth=0.5)  # Lidar lines in red
        if d < min_dist_obstacle and d < lidar_range: # Consider only detected obstacles
            min_dist_obstacle = d
            min_angle_deg_obstacle = np.degrees(a_rad)

    # Display obstacle information
    obstacle_text = f"Obstacle: Dist={min_dist_obstacle:.2f}m, Angle={min_angle_deg_obstacle:.1f}deg"
    plt.text(xmin + 5, ymax - 1, obstacle_text, fontsize=8, verticalalignment='top', horizontalalignment='left')

    # Display drone's state information
    vx, vy = state[3], state[4]
    ax_drone, ay_drone = state[6], state[7]
    state_text = f"Pos:({drone_x:.2f},{drone_y:.2f}), Speed:{np.linalg.norm([vx, vy]):.2f}m/s, Accel:({ax_drone:.2f},{ay_drone:.2f})m/s^2"
    plt.text(xmin + 5, ymin + 5, state_text, fontsize=8, verticalalignment='top', horizontalalignment='left')

    plt.pause(0.01)  # Pause for a moment to update the plot

    
""" # Run simulation
plt.figure()
for t in range(steps):
    lidar_data = simulate_lidar_2d(state)
    
    control = evasion_strategy(lidar_data, warning_distance, danger_distance, a_max)    
    state = update_state(state, control)
     
    plot_scene(state, map_bounds, lidar_data)
    x, y, z = state[0], state[1], state[2]
    if not (xmin <= x <= xmax and ymin <= y <= ymax):
        print(f"🚨 Crash detected at position ({x:.2f}, {y:.2f})")
        break
    

plt.show()
 """