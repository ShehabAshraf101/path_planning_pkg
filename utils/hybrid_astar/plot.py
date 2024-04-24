import matplotlib.pyplot as plt
import numpy as np

# Define the size of the map
map_width = 45  # in meters
map_height = 45  # in meters

# Define obstacles (center_x, center_y, length, width)
ob1 = (21.9, 18.0, 3.0, 27.0)
ob2 = (29.4, 33.0, 18.0, 3.0)
ob3 = (10.5, 22.5, 3.0, 36.0)
ob4 = (24.0, 42.0, 30.0, 3.0)
ob5 = (18.0, 22.8, 3.0, 2.4)
ob6 = (14.25, 28.5, 1.5, 4.8)
ob7 = (18.0, 34.8, 3.0, 2.4)
# obstacles = [] # No obstacles
# obstacles = [ob1, ob2, ob3, ob4] # With drivable area only
obstacles = [ob1, ob2, ob3, ob4, ob5, ob6, ob7] # Driveable area and obstacles

# Create the plot with map and obstacles
fig, ax = plt.subplots()

# Plot the map boundary
ax.plot([0, map_width, map_width, 0, 0], [0, 0, map_height, map_height, 0], 'k-')

# Plot the obstacles
for obstacle in obstacles:
    center_x, center_y, length, width = obstacle
    x = center_x - length / 2
    y = center_y - width / 2
    rectangle = plt.Rectangle((x, y), length, width, color='black')
    ax.add_patch(rectangle)

# Define functions for plotting arrows to show heading direction
def plot_arrows(pose, axes):
    color = 'black'
    length = 0.3

    for i in range(0, pose.shape[0]):
        delta = np.zeros_like(pose[i, 0:2])
        delta[0] = length * np.cos(pose[i, 2])
        delta[1] = length * np.sin(pose[i, 2])

        axes.arrow(pose[i, 0], pose[i, 1], delta[0], delta[1], head_width=0.1, head_length=0.15, color=color)

# Define 2D pose numpy array and plot it onto figure
path = np.array([[18, 18, 1.5708], [17.8135, 18.7249, 1.68927], [17.3796, 19.3329, 1.94619], [16.9563, 19.9504, 2.06466], [16.6276, 20.6234, 2.06466], [16.2988, 21.2963, 2.06466], [16.1349, 22.0267, 1.94619], [16.2445, 22.7656, 1.68927], [16.5416, 23.4509, 1.43236], [16.6583, 24.1908, 1.43236], [16.775, 24.9306, 1.43236], [16.8917, 25.6705, 1.43236], [17.0084, 26.4104, 1.43236], [17.1251, 27.1502, 1.43236], [17.2418, 27.8901, 1.43236], [17.1839, 28.6364, 1.55084], [17.1704, 29.3853, 1.55084], [17.1568, 30.1341, 1.55084], [17.1432, 30.883, 1.55084], [17.1297, 31.6319, 1.55084], [16.9432, 32.3568, 1.66931], [16.8644, 33.1017, 1.66931], [16.7856, 33.8465, 1.66931], [16.5366, 34.5525, 1.78779], [16.4705, 35.2975, 1.53088], [16.5958, 36.0348, 1.27396], [16.9044, 36.7161, 1.01705], [17.376, 37.2967, 0.760134], [17.9796, 37.7383, 0.50322], [18.6756, 38.0121, 0.246306], [19.4184, 38.1001, -0.010608], [20.4216, 37.9109, -0.362141], [21.123, 37.6452, -0.362141], [21.8243, 37.3795, -0.362141], [22.5257, 37.1138, -0.362141], [23.227, 36.8481, -0.362141], [23.9284, 36.5823, -0.362141], [24.9658, 36.1893, -0.362141], [26, 36, 0]])

ax.scatter(path[:, 0], path[:, 1], label=f"Planned Path", marker='x', color='red', s=10)
plot_arrows(path, ax)

# Set limits for both axes
ax.set_xlim(0, map_width)
ax.set_ylim(0, map_height)
ax.set_aspect('equal')
plt.grid(True)

# Set the title and axis labels
plt.title("Hybrid A* Search")
plt.xlabel("X-axis (m)")
plt.ylabel("Y-axis (m)")

# Show the plot
# plt.tight_layout()
plt.legend()
plt.show()