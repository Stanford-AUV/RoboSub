import numpy as np
from scipy.interpolate import make_interp_spline
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Example points (x, y)
x = np.random.rand(5)
y = np.random.rand(5)
x = np.concatenate((x, [x[0]]))
y = np.concatenate((y, [y[0]]))

# Parameter t (can be the distance between the points or just range of integers)
t = np.linspace(0, 1, len(x))

# Fit cubic splines to x and y as a function of t
spline_x = make_interp_spline(
    t, x, k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)])
)
spline_y = make_interp_spline(
    t, y, k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)])
)

# Define a finer grid of t values for evaluation
t_fine = np.linspace(0, 1, 100)

# Compute positions
x_fine = spline_x(t_fine)
y_fine = spline_y(t_fine)

# Compute velocities (first derivative)
v_x = spline_x(t_fine, 1)
v_y = spline_y(t_fine, 1)

# Compute accelerations (second derivative)
a_x = spline_x(t_fine, 2)
a_y = spline_y(t_fine, 2)

# Plot the results
plt.figure(figsize=(10, 8))

# Plot the path
plt.subplot(4, 1, 1)
plt.plot(x_fine, y_fine, label="Path (x, y)")
plt.scatter(x, y, color="red", label="Points")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()

# Plot position
plt.subplot(4, 1, 2)
plt.plot(t_fine, x_fine, label="x")
plt.plot(t_fine, y_fine, label="y")
plt.xlabel("t")
plt.ylabel("Position")
plt.legend()

# Plot velocity
plt.subplot(4, 1, 3)
plt.plot(t_fine, v_x, label="v_x")
plt.plot(t_fine, v_y, label="v_y")
plt.xlabel("t")
plt.ylabel("Velocity")
plt.legend()

# Plot acceleration
plt.subplot(4, 1, 4)
plt.plot(t_fine, a_x, label="a_x")
plt.plot(t_fine, a_y, label="a_y")
plt.xlabel("t")
plt.ylabel("Acceleration")
plt.legend()

plt.tight_layout()
plt.show()


# Setup the figure and axis
fig, ax = plt.subplots()
ax.plot(x_fine, y_fine, label="Path (x, y)")
ax.scatter(x, y, color="red", label="Points")
(vehicle,) = ax.plot([], [], "bo", markersize=8, label="Vehicle")

# Set up the plot limits and labels
ax.set_xlim(np.min(x_fine) - 0.5, np.max(x_fine) + 0.5)
ax.set_ylim(np.min(y_fine) - 0.5, np.max(y_fine) + 0.5)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.legend()


# Initialization function: plot the background of each frame
def init():
    vehicle.set_data([], [])
    return (vehicle,)


# Animation function: this is called sequentially
def animate(i):
    # Update the vehicle position at frame i
    vehicle.set_data(x_fine[i], y_fine[i])
    return (vehicle,)


# Create the animation
anim = FuncAnimation(
    fig, animate, init_func=init, frames=len(t_fine), interval=20, blit=True
)

# Show the animation
plt.show()
