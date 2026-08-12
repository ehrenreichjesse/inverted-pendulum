import control
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# constants
m = 0.055
M = 0.14
g = 9.81
L = 0.3
I = 1/3*m*L**2
dt = 0.02
t_max = 2

# initial conditions
theta = np.radians(20)
theta_dot = 0
base_dot = 0
base = 0

D = I*(M + m) + M*m*L**2
a_23 = m**2*g*L**2/D
a_43 = m*g*L*(M + m)/D
b_21 = (I + m*L**2)/D
b_41 = m*L/D
A = np.array([[0, 1, 0, 0], [0, 0, a_23, 0], [0, 0, 0, 1], [0, 0, a_43, 0]])
B = np.array([[0], [b_21], [0], [b_41]])
Q = np.diag([44.4, 1, 2.8, 0.25])
R = 0.05

K, S, E = control.lqr(A, B, Q, R)
K = K.flatten()
x = np.array([base, base_dot, theta, theta_dot])

theta_list = []
time_list = []
base_list = []

for t in np.arange(0, t_max, dt):
    # control input
    u = np.dot(-K, x)
    
    # state space dynamics
    x_dot = A @ x + B.flatten() * u
    x = x + x_dot * dt

    # unpack states
    base = -x[0]
    theta = x[2]

    # store
    theta_list.append(theta)
    time_list.append(t)
    base_list.append(base)

# convert to positions
base = base_list
xDir = L*np.sin(theta_list) + base_list
yDir = L*np.cos(theta_list)

# animation
fig, ax = plt.subplots()

scale = 1
ax.set_xlim(-scale*L, scale*L)
ax.set_ylim(-scale*L, scale*L*1.2)
ax.set_aspect('equal')

# plot elements
pendulum_line, = ax.plot([], [], marker = 'o', lw = 3)


def update(frame):
    # update pendulum line
    pendulum_line.set_data([base[frame], xDir[frame]], [0, yDir[frame]])
    return pendulum_line,

ani = FuncAnimation(fig, update, frames = len(xDir), interval = dt*1000)

# plot
plt.title("Falling Pendulum")
plt.show()

print("A =", A)
print("B =", B)
print("K =", K)

print("D =", D)
print("I =", I)

print("Q = ", Q)
print("R = ", R)

print("K =", K.flatten())
print("Eigenvalues of open loop A:", np.linalg.eigvals(A))
K_flat = K.flatten()
A_cl = A - B.flatten().reshape(4,1) @ K_flat.reshape(1,4)
print("Closed loop A:", A_cl)
print("Eigenvalues of closed loop:", np.linalg.eigvals(A_cl))