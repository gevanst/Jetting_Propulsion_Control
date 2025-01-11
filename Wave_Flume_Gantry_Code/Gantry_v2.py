import numpy as np
import matplotlib.pyplot as plt

# Constants
a_mst_mm = 144.2  # microstep/mm
a_mm_m = 1000  # mm/m
a_mst_m = a_mst_mm * a_mm_m  # microstep/m

mst_total = 500000  # total microsteps

# Every 10 microsteps
x_pos = np.arange(0, mst_total, 10)  # (microsteps) x position
x_vel = np.zeros(len(x_pos))  # (microsteps/s) x velocity

# Desired velocities
desired_ss_velocity = 0.1  # (m/s) desired steady-state velocity
xv_ss = desired_ss_velocity * a_mst_m  # (microsteps/s)
desired_start_velocity = 0.1  # (m/s) desired start velocity
xv_start = desired_start_velocity * a_mst_m  # (microsteps/s)

# Ramp-up and ramp-down regions
ramp_up = int(0.2 * len(x_pos))  # 
ramp_down = int(0.8 * len(x_pos))  #

# Velocity profile generation
for i in range(len(x_pos)):
    if i < ramp_up:  # Ramp-up region
        x_vel[i] = i * ((xv_ss - xv_start) / ramp_up) + xv_start
    elif i > ramp_down:  # Ramp-down region
        pos = (i - ramp_down) / (len(x_pos) - ramp_down)  # Normalize ramp-down position to [0, 1]
        # Quadratic function for ramp-down: v = v_ss * (1 - pos^2)
        scale_factor = 1 - pos**2
        x_vel[i] = xv_ss * scale_factor
    else:  # Steady-state region
        x_vel[i] = xv_ss

# Normalize y-axis to m/s
x_vel_m = x_vel / a_mst_m

# Normalize x-axis to m
x_pos_m = x_pos / a_mst_m

# Calculate time intervals using dx = v * dt
dx = np.diff(x_pos_m, prepend=0)  # Position increments
dt = dx / np.maximum(x_vel_m, 1e-6)  # Prevent division by zero
time = np.cumsum(dt)  # Cumulative time

# Plot velocity vs position
plt.figure(figsize=(10, 5))
plt.plot(x_pos_m, x_vel_m, label="Velocity vs Position")
plt.xlabel('Position (m)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Profile with Quadratic Ramp-Down')
plt.grid(True)
plt.legend()
plt.show()

# Plot velocity vs time
plt.figure(figsize=(10, 5))
plt.plot(time, x_vel_m, label="Velocity vs Time", color="orange")
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity vs Time')
plt.grid(True)
plt.legend()
plt.show()

# Generate other velocity components
y_vel = np.zeros(len(x_pos))
z_vel = np.zeros(len(x_pos))

x_pos += 1  # Increment position for the text file

# Save data to a text file
data = np.array([x_pos, x_vel, y_vel, z_vel])
np.savetxt('0_1m_s_quad_500k.txt', data, delimiter='\t', fmt='%i', newline='\r')
