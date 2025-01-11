import numpy as np
import matplotlib.pyplot as plt

a_mst_mm = 144.2 # microstep/mm
a_mm_m = 1000 # mm/m

a_mst_m = a_mst_mm * a_mm_m # microstep/m
mst_total = 500000 # total microsteps

# every 10 mst
x_pos = np.arange(0, mst_total, 10) # (microsteps) x position

x_vel = np.zeros(len(x_pos)) # (microsteps/s) x velocity

desired_ss_velocity = 1.0 # (m/s) desired steady state velocity
xv_ss = desired_ss_velocity * a_mst_m # (microsteps/s)
desired_start_velocity = 0.1 # (m/s) desired start velocity
xv_start = desired_start_velocity * a_mst_m # (microsteps/s)

ramp_up =  0.2 * len(x_pos)
ramp_down = 0.45 * len(x_pos)

steepness = 6
offset = 0.5

for i in range(len(x_pos)):
    # linear vel vs time rampup
    if i < ramp_up:
        x_vel[i] = i * ((xv_ss - xv_start) / ramp_up) + xv_start
    elif i > ramp_down:
        x_vel[i] = xv_ss - (i - ramp_down) * (xv_ss / (len(x_pos) - ramp_down)) #linear ramp down vel vs position
        # pos = (i - ramp_down) / (len(x_pos) - ramp_down) #normalize between the rampdown start and end position
        # scale_factor = (np.tanh(steepness * (pos-offset))+1)/2 # Tanh ramp down
        # x_vel[i] = xv_ss * (1 - scale_factor)
    else:
        x_vel[i] = xv_ss

# normalize y-axis to m/s
x_vel_m = x_vel / a_mst_m
# normalize x-axis to m
x_pos_m = x_pos / a_mst_m


dx = np.diff(x_pos_m, prepend=0)  # Position increments
dt = dx / np.maximum(x_vel_m, 1e-6)  # Prevent division by zero
time = np.cumsum(dt)  # Cumulative time


plt.plot(x_pos_m, x_vel_m)
plt.xlabel('Position (m)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Profile')
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

y_vel = np.zeros(len(x_pos))
z_vel = np.zeros(len(x_pos))

x_pos = x_pos+1

# make a text document, tab delimited between entries within arrays, with 4 rows of x_pos, x_vel, y_vel, z_vel and CRLF at the end of each line
# should only have four rows and mst_total columns
data = np.array([x_pos, x_vel, y_vel, z_vel])
np.savetxt('1_0m_s_tanh.txt', data, delimiter='\t', fmt='%i', newline='\r')
