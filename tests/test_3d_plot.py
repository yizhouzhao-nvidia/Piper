import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Initialize the figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initial data
num_points = 10
xs = np.random.rand(num_points)
ys = np.random.rand(num_points)
zs = np.random.rand(num_points)

# Create the scatter plot
scatter = ax.scatter(xs, ys, zs, c='r', marker='o')

# Set the limits of the plot
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)

# Animation update function
def update(frame):
    # Simulate new data
    new_xs = np.random.rand(num_points)
    new_ys = np.random.rand(num_points)
    new_zs = np.random.rand(num_points)

    # Update scatter plot
    scatter._offsets3d = (new_xs, new_ys, new_zs)

    # Optionally return scatter
    return scatter,

# Create the animation
ani = FuncAnimation(fig, update, frames=100, interval=100, blit=False)

plt.show()
