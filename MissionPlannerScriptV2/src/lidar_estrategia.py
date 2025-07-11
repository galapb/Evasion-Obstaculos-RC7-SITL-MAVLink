
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from UAV_3D_Simulation import( A, B, W, a_noise_std,
    map_bounds, walls, lidar_range, lidar_resolution_deg, num_rays, 
    sensor_angles, sensor_noise_cm, xmin, xmax,ymin, ymax,warning_distance, 
    danger_distance, 
    mass, v_max, a_max, max_thrust, base_thrust, 
    max_pitch_roll, max_yaw_rate,
    simulate_lidar_2d, evasion_strategy, update_state, plot_scene)


# Simulation parameters
dt = 0.1  # time step
T = 60.0  # total simulation time
steps = int(T / dt)

# UAV state vector: [x, y, z, vx, vy, vz, ax, ay, az]
state = np.zeros(9)
state[:3] = [-20, 0.0, 0.0]  # initial position
state[3:6] = [2.0, 2.0, 0.0]  # vx = 1 m/s to the right



# Run simulation
plt.figure()
for t in range(steps):
    lidar_data = simulate_lidar_2d(state) #Esto tiene que ser una tupla -- (distancia, angulo)
    
    control = evasion_strategy(lidar_data, warning_distance, danger_distance, a_max)    
    state = update_state(state, control)
     
    plot_scene(state, map_bounds, lidar_data)
    x, y, z = state[0], state[1], state[2]
    if not (xmin <= x <= xmax and ymin <= y <= ymax):
        print(f"🚨 Crash detected at position ({x:.2f}, {y:.2f})")
        break
    

plt.show()