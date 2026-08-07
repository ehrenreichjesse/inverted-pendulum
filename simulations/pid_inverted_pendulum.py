import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# constants
m = 0.018
g = 9.81
L = 0.17
dt = 0.02
t_max = 5

Kp = 70
Kd = 4
Kp_base = 20
Kd_base = 10

# initial conditions
theta = np.radians(30)
theta_dot = 0
base_dot = 0
base = 0

theta_list = []
time_list = []
base_list = []

for t in np.arange(0, t_max, dt):
    base_ddot = Kp*theta + Kd*theta_dot + Kp_base*base + Kd_base*base_dot

    base_dot += base_ddot*dt
    base += base_dot*dt
    
    # compute acceleration
    theta_ddot = 3/L/2*(g*np.sin(theta) - base_ddot*np.cos(theta))

    # integrate
    theta_dot += theta_ddot*dt
    theta += theta_dot*dt

    # store
    theta_list.append(theta)
    time_list.append(t)
    base_list.append(base)

# convert to positions
base = base_list
x = L*np.sin(theta_list) + base_list
y = L*np.cos(theta_list)

# animation
fig, ax = plt.subplots()

scale = 4
ax.set_xlim(-scale*L, scale*L)
ax.set_ylim(-scale*L, scale*L)
ax.set_aspect('equal')

# plot elements
pendulum_line, = ax.plot([], [], marker = 'o', lw = 3)


def update(frame):
    # update pendulum line
    pendulum_line.set_data([base[frame], x[frame]], [0, y[frame]])
    return pendulum_line,

ani = FuncAnimation(fig, update, frames = len(x), interval = dt*1000)

# plot
plt.title("Falling Pendulum")
plt.show()

