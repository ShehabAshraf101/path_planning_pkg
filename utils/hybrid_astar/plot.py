import matplotlib.pyplot as plt
import numpy as np

# Define the size of the map
map_width = 45  # in meters
map_height = 45  # in meters

# Define obstacles (center_x, center_y, length, width)
ob1 = (21.9, 18.0, 1.8, 28.2)
ob2 = (29.4, 33.0, 16.8, 1.8)
ob3 = (10.5, 22.5, 1.8, 37.2)
ob4 = (24.0, 42.0, 28.8, 1.8)
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
path = np.array([[18, 18, 1.5708], [17.8578, 18.7351, 1.65863], [17.5191, 19.402, 1.8424], [17.0698, 19.9999, 2.02618], [16.6302, 20.6061, 2.11401], [16.0414, 21.0673, 2.29778], [15.3815, 21.4194, 2.48156], [14.7269, 21.783, 2.56939], [14.1057, 22.2014, 2.56939], [13.5659, 22.7203, 2.48156], 
[13.1903, 23.3671, 2.29778], [12.9326, 24.0693, 2.11401], [12.6627, 24.7677, 2.02618], [12.5956, 25.5126, 1.8424], [12.6588, 26.2579, 1.65863], [12.58, 27.0027, 1.65863], [12.5013, 27.7476, 1.65863], [12.5518, 28.4946, 1.5708], [12.5383, 29.2435, 1.5708], [12.3961, 29.9786, 1.65863], 
[12.3173, 30.7235, 1.65863], [12.3679, 31.4705, 1.5708], [12.6218, 32.174, 1.38702], [12.9941, 32.8227, 1.20325], [13.4734, 33.397, 1.01947], [13.8918, 34.0182, 1.01947], [14.3103, 34.6394, 1.01947], [14.6154, 35.3232, 1.1073], [14.9781, 35.9785, 1.1073], [15.448, 36.5614, 1.01947], 
[16.0596, 36.992, 0.835694], [16.7366, 37.3099, 0.651918], [17.4089, 37.6396, 0.564088], [18.1073, 37.9095, 0.476257], [18.8523, 37.9766, 0.292482], [19.5975, 37.9134, 0.108706], [20.3464, 37.926, -0.0750697], [21.7677, 37.5539, -0.437053], [22.4472, 37.2364, -0.437053], [23.1267, 36.919, -0.437053], 
[24.2726, 36.3836, -0.437054], [24.9774, 36.1302, -0.253278], [26, 36, 0]])

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