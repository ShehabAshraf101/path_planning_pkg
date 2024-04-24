import numpy as np
import matplotlib.pyplot as plt

# Defining vehicle constant parameters
wheelbase = 2.269
rear_to_cg = 1.1
steering = np.array([-np.pi/6, -np.pi/9, -np.pi/18, 0.0, np.pi/18, np.pi/9, np.pi/6])
beta = np.arctan2((rear_to_cg/wheelbase) * np.tan(steering), 1)
curvature = np.cos(beta) * np.tan(steering)/wheelbase
ts = 0.5

path = np.array([[0, 0], [0.472066, 0.163833], [0.928057, 0.368185], [1.34163, 0.648616], [1.7292, 0.964024], [2.0561, 1.34194], [2.34882, 1.74691], [2.65475, 2.14223], [2.92506, 2.56271], [3.15769, 3.00515], [3.35087, 3.46619], [3.55428, 3.9229], [3.7577, 4.37962], [3.92053, 4.85233], [4.08336, 5.32504], [4.20438, 5.81014], [4.37539, 6.27999], [4.5464, 6.74984], [4.71741, 7.21968], [4.88842, 7.68953], [5.05943, 8.15938], [5.18044, 8.64448], [5.25872, 9.13828], [5.33699, 9.63209], [5.37193, 10.1308], [5.35217, 10.6303], [5.28896, 11.1262], [5.18277, 11.6146], [5.03441, 12.092], [4.89666, 12.5726], [4.75891, 13.0532], [4.5798, 13.52], [4.40068, 13.9868]])

# Predict motion using euler's method
def sim_model_euler(x_curr, beta, curv):
    dt = 0.001
    x = np.zeros((x_curr.shape[0], beta.shape[0] + 1))
    x_full = np.zeros((x_curr.shape[0], (beta.shape[0] * int(ts/dt)) + 1))
    x[:, 0] = x_curr
    x_full[:, 0] = x_curr
    num_updates = int(ts/dt)
    for i in range(0, beta.shape[0]):
        x[:, i+1] = x[:, i]
        for t in range(0, num_updates):
            offset = np.array([np.cos(beta[i] + x[2, i+1]), np.sin(beta[i] + x[2, i+1]), curv[i]])
            x[:, i+1] += dt * offset
            x_full[:, i * num_updates + t + 1] = x[:, i+1]

    return x, x_full

# Function for limiting angles to to -pi to pi
def wrap_pi(angle):
    mod_angle = np.mod(angle, 2 * np.pi)
    if mod_angle > np.pi:
        return mod_angle - 2 * np.pi
    elif mod_angle < -np.pi:
        return mod_angle + 2 * np.pi
    else:
        return mod_angle
    
# Function for rounding orientations to nearest discrete value
precision = np.deg2rad(5.0)
def round_to_nearest(value):
    return np.round(value/precision) * precision

# Calculate offset using euler for a steering angle and orientation pair
def get_offset(input_index, orientation):
    offset_x = 0
    offset_y = 0
    ds = 0
    dt = 0.001
    theta_curr = orientation
    num_updates = int(ts/dt)
    for t in range(0, num_updates):
        dx = dt * np.cos(beta[input_index] + theta_curr)
        dy = dt * np.sin(beta[input_index] + theta_curr)
        ds += np.sqrt(dx * dx + dy * dy)
        offset_x += dx
        offset_y += dy
        theta_curr += dt * curvature[input_index]
    
    return offset_x, offset_y, ds

# Compute a lookup table of movements for each orientation and steering pair
theta = np.arange(-np.pi, np.pi, precision)
delta_theta = ts * curvature
delta_theta_index = (np.round(delta_theta/precision)).astype(int)
print(delta_theta_index)
delta_x = np.zeros((steering.shape[0], theta.shape[0]))
delta_y = np.zeros((steering.shape[0], theta.shape[0]))
for i in range(0, steering.shape[0]):
    for j in range(0, theta.shape[0]):
        delta_x[i, j], delta_y[i, j], ds = get_offset(i, theta[j])
        # print(ds)

def sim_model_alt(x_curr, input_indices):
    x = np.zeros((x_curr.shape[0], input_indices.shape[0] + 1))
    x[:, 0] = x_curr
    theta_rounded = round_to_nearest(x[2, 0])
    theta_index = int((theta_rounded - theta[0])/precision)
    
    for i in range(0, input_indices.shape[0]):
        # Round the orientation to the nearest value
        theta_rounded = round_to_nearest(x[2, i])
        theta_index = int((theta_rounded - theta[0])/precision)
        # theta_index = 0 if theta_index >= theta.shape[0] else theta_index
        offset = np.array([delta_x[input_indices[i], theta_index], delta_y[input_indices[i], theta_index], delta_theta[input_indices[i]]])
        x[:, i+1] = x[:, i] + offset
        x[2, i+1] = wrap_pi(x[2, i+1])
        # theta_index += delta_theta_index[input_indices[i]]

    return x


if __name__ == "__main__":
    x_start = np.array([0.0, 0.0, np.deg2rad(0)])
    u_steer = np.array([steering[-1], steering[-1], steering[-1], steering[-1], steering[-1], steering[-1], 
                        steering[-2], steering[-2], steering[-2], steering[-2], steering[-3], steering[-3], steering[-3], steering[-3], steering[-3],
                        steering[3], steering[3], steering[3], steering[3], steering[3], steering[4], steering[4], steering[4], steering[4],
                        steering[5], steering[5], steering[5], steering[5], steering[4], steering[4], steering[4], steering[4]])
    beta = np.arctan2((rear_to_cg/wheelbase) * np.tan(u_steer), 1)
    curv = np.cos(beta) * np.tan(u_steer)/wheelbase
    input_indices = np.array([-1, -1, -1, -1, -1, -1, 
                              -2, -2, -2, -2, -3, -3, -3, -3, -3,
                              3, 3, 3, 3, 3, 4, 4, 4, 4,
                              5, 5, 5, 5, 4, 4, 4, 4], dtype=np.int32)

    x, x_full = sim_model_euler(x_start, beta, curv)
    x_alt = sim_model_alt(x_start, input_indices)

    # Plot the results
    figure, axes = plt.subplots()
    axes.scatter(x[0, :], x[1, :], color='red', marker='o', s=50, label="Path using Euler")
    axes.scatter(x_alt[0, :], x_alt[1, :], color='green', marker='x', s=50, label="Path using approximate method (Python)")
    axes.scatter(path[:, 0], path[:, 1], color='blue', marker='x', s=50, label="Path using approximate method (C++)")
    # axes.plot(x_full[0, :], x_full[1, :], color='blue', label="Path using Euler (detailed)")
    axes.set_xlabel('x-axis (m)')
    axes.set_ylabel('y-axis (m)')
    axes.axis('equal')
    axes.legend()
    axes.grid(True)

    plt.show()