import numpy as np
import matplotlib.pyplot as plt

# Constants
a_mst_mm = 144.2  # microstep/mm
a_mm_m = 1000  # mm/m
a_mst_m = a_mst_mm * a_mm_m  # microstep/m

mst_total = 500000  # total microsteps
desired_ss_velocity = 0.25  # (m/s) desired steady-state velocity
xv_ss = -desired_ss_velocity * a_mst_m  # Negative velocity (microsteps/s)
desired_start_velocity = 0.1  # (m/s) start velocity
xv_start = -desired_start_velocity * a_mst_m  # Negative start velocity (microsteps/s)

# Create position array from 500,000 to 0
x_pos = np.arange(mst_total, 0, -10)  # Microsteps from 500,000 to 0
x_vel = np.zeros(len(x_pos))  # Velocity profile

# Ramp-up and ramp-down lengths (percentage of total motion)
ramp_up_len = int(0.2 * len(x_pos))  # 20% of positions for ramp-up
ramp_down_start = int(0.8 * len(x_pos))  # Ramp-down starts at 80%

# Velocity profile generation
for i in range(len(x_pos)):
    if i < ramp_up_len:  # Ramp-up region (negative velocity increasing in magnitude)
        x_vel[i] = i * ((xv_ss - xv_start) / ramp_up_len) + xv_start
    elif i >= ramp_down_start:  # Ramp-down region (negative velocity decreasing in magnitude)
        pos = (i - ramp_down_start) / (len(x_pos) - ramp_down_start)  # Normalize to [0, 1]
        x_vel[i] = xv_ss * (1 - pos)  # Linearly decrease velocity to 0
    else:  # Steady-state region
        x_vel[i] = xv_ss

# Normalize y-axis to m/s
x_vel_m = x_vel / a_mst_m

# Normalize x-axis to m
x_pos_m = x_pos / a_mst_m

# Calculate time intervals using dx = v * dt
dx = np.abs(np.diff(x_pos_m, prepend=0))  # Position increments
dt = dx / np.maximum(np.abs(x_vel_m), 1e-6)  # Prevent division by zero
time = np.cumsum(dt)  # Cumulative time

# Plot velocity vs position
plt.figure(figsize=(10, 5))
plt.plot(x_pos_m, x_vel_m, label="Velocity vs Position")
plt.xlabel('Position (m)')
plt.ylabel('Velocity (m/s)')
plt.title('Inverted Velocity Profile for Gantry')
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

# Save data to a text file
data = np.array([x_pos, x_vel, y_vel, z_vel])
np.savetxt('gantry_return_500k.txt', data, delimiter='\t', fmt='%i', newline='\r')
