import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch

path = np.array([[2.49893e-16, 0, 0], [0.49875, -0.030591, -0.122517], [0.990023, -0.121905, -0.245034], [1.46645, -0.272574, -0.367551], [1.9209, -0.480338, -0.490068], [2.34655, -0.742083, -0.612585], [2.73702, -1.05389, -0.735102], [3.08646, -1.41107, -0.857619], [3.38962, -1.80828, -0.980137], [3.73565, -2.4379, -1.15641], [3.93697, -2.89558, -1.15641], [4.13828, -3.35326, -1.15641], [4.3396, -3.81094, -1.15641], [4.54091, -4.26862, -1.15641], [4.74223, -4.7263, -1.15641], [4.94355, -5.18399, -1.15641], [5.14486, -5.64167, -1.15641], [5.34618, -6.09935, -1.15641], [5.54749, -6.55703, -1.15641], [5.74881, -7.01471, -1.15641], [5.95012, -7.47239, -1.15641], [6.15144, -7.93007, -1.15641], [6.35276, -8.38775, -1.15641], [6.55407, -8.84543, -1.15641], [6.75539, -9.30312, -1.15641], [6.9567, -9.7608, -1.15641], [7.15802, -10.2185, -1.15641], [7.35933, -10.6762, -1.15641], [7.56065, -11.1338, -1.15641], [7.76197, -11.5915, -1.15641], [7.96328, -12.0492, -1.15641], [8.1646, -12.5069, -1.15641], [8.36591, -12.9646, -1.15641], [8.56723, -13.4222, -1.15641], [8.76854, -13.8799, -1.15641], [8.96986, -14.3376, -1.15641], [9.17118, -14.7953, -1.15641], [9.37249, -15.253, -1.15641], [9.57381, -15.7107, -1.15641], [9.77512, -16.1683, -1.15641], [9.97644, -16.626, -1.15641], [10.1778, -17.0837, -1.15641], [10.3791, -17.5414, -1.15641], [10.5804, -17.9991, -1.15641], [10.7817, -18.4567, -1.15641], [10.983, -18.9144, -1.15641], [11.1843, -19.3721, -1.15641], [11.3856, -19.8298, -1.15641], [11.587, -20.2875, -1.15641], [11.7883, -20.7451, -1.15641], [12.1833, -21.6432, -1.15641], [12.4121, -22.0874, -1.03389], [12.6935, -22.5003, -0.911372], [13.0232, -22.8758, -0.788855], [13.3964, -23.2081, -0.666338], [13.8074, -23.4923, -0.543821], [14.25, -23.7242, -0.421304], [14.7176, -23.9002, -0.298787], [15.2033, -24.0178, -0.17627], [15.6997, -24.0752, -0.0537529], [16.1993, -24.0714, 0.0687642], [16.6948, -24.0066, 0.191281], [17.1787, -23.8818, 0.313798], [17.6436, -23.6987, 0.436315], [18.0827, -23.4602, 0.558832], [18.4894, -23.1699, 0.68135], [18.8575, -22.832, 0.803867], [19.1816, -22.4516, 0.926384], [19.4567, -22.0345, 1.0489], [19.6788, -21.5869, 1.17142], [19.8446, -21.1155, 1.29393], [19.9515, -20.6274, 1.41645], [20, -20, 1.5708]])

def generate_dubins_path(path_type, r_min, x_start, x_goal):
    # Get the locations of both circles' centerpoints
    x_center1 = np.array([0.0, 0.0])
    x_center2 = np.array([0.0, 0.0])

    if (path_type[0] == 'R'):
        x_center1[0] = x_start[0] + r_min * np.sin(x_start[2])
        x_center1[1] = x_start[1] - r_min * np.cos(x_start[2])
    else:
        x_center1[0] = x_start[0] - r_min * np.sin(x_start[2])
        x_center1[1] = x_start[1] + r_min * np.cos(x_start[2])

    if (path_type[2] == 'R'):
        x_center2[0] = x_goal[0] + r_min * np.sin(x_goal[2])
        x_center2[1] = x_goal[1] - r_min * np.cos(x_goal[2])
    else:
        x_center2[0] = x_goal[0] - r_min * np.sin(x_goal[2])
        x_center2[1] = x_goal[1] + r_min * np.cos(x_goal[2])

    # Get common parameters
    delta_center = x_center2 - x_center1
    dist = np.sqrt(np.sum(np.power(delta_center, 2)))
    theta = np.arctan2(delta_center[1], delta_center[0])

    # Get path dependent parameters and generate path
    if (path_type == "RSR"):
        theta_1, theta_2, combined_arc_length = get_params_rsr(r_min, theta, x_start, x_goal)
        path_x, path_y, path_length = sample_dubins_path(combined_arc_length, r_min, x_center1, theta_1, True, x_center2, theta_2, True)
        print(f"Dubins RSR Path:")
        print(f"Path Length 1 = {combined_arc_length + dist:.4f} m")
    elif (path_type == "RSL"):
        theta_1, theta_2, combined_arc_length, length_st = get_params_rsl(r_min, dist, theta, x_start, x_goal, x_center1, x_center2)
        path_x, path_y, path_length = sample_dubins_path(combined_arc_length, r_min, x_center1, theta_1, True, x_center2, theta_2, False)
        print(f"Dubins RSL Path:")
        print(f"Path Length 1 = {combined_arc_length + length_st:.4f} m")
    elif (path_type == "LSR"):
        theta_1, theta_2, combined_arc_length, length_st = get_params_lsr(r_min, dist, theta, x_start, x_goal, x_center1, x_center2)
        path_x, path_y, path_length = sample_dubins_path(combined_arc_length, r_min, x_center1, theta_1, False, x_center2, theta_2, True)
        print(f"Dubins LSR Path:")
        print(f"Path Length 1 = {combined_arc_length + length_st:.4f} m")
    else:
        theta_1, theta_2, combined_arc_length = get_params_lsl(r_min, theta, x_start, x_goal)
        path_x, path_y, path_length = sample_dubins_path(combined_arc_length, r_min, x_center1, theta_1, False, x_center2, theta_2, False)
        print(f"Dubins LSL Path:")
        print(f"Path Length 1 = {combined_arc_length + dist:.4f} m")
    
    print(f"Start Angle = {np.rad2deg(theta_1[0]):.4f} deg \n", f"First Tangent = {np.rad2deg(theta_1[0] + theta_1[1]):.4f} deg \n",
        f"Second Tangent = {np.rad2deg(theta_2[0]):.4f} deg \n", f"Final Angle = {np.rad2deg(theta_2[0] + theta_2[1]):.4f} deg")
    print(f"Path Length = {path_length:.4f} m")

    return path_x, path_y, x_center1, x_center2

def get_params_rsr(r_min, theta, xs, xg):
    theta_s  = np.pi/2 + xs[2]
    theta_t1 = np.pi/2 + theta
    theta_t2 = np.pi/2 + theta
    theta_g  = np.pi/2 + xg[2]

    delta_theta1 = theta_t1 - theta_s
    delta_theta2 = theta_g - theta_t2

    if (delta_theta1 > 0):
        delta_theta1 -= 2 * np.pi
    if (delta_theta2 > 0):
        delta_theta2 -= 2 * np.pi

    combined_arc_length = r_min * -(delta_theta1 + delta_theta2)

    return np.array([theta_s, delta_theta1]), np.array([theta_t2, delta_theta2]), combined_arc_length

def get_params_rsl(r_min, dist, theta, xs, xg, x_cs, x_cg):
    theta_s  = np.pi/2 + xs[2]
    theta_t1 = np.arccos(2 * r_min/dist) + theta
    theta_t2 = np.arccos(2 * r_min/dist) + theta - np.pi
    theta_g  = -np.pi/2 + xg[2]

    delta_theta1 = theta_t1 - theta_s
    delta_theta2 = theta_g - theta_t2

    if (delta_theta1 > 0):
        delta_theta1 -= 2 * np.pi
    if (delta_theta2 < 0):
        delta_theta2 += 2 * np.pi

    combined_arc_length = r_min * (-delta_theta1 + delta_theta2)

    start_st = np.array([0.0, 0.0])
    end_st = np.array([0.0, 0.0])
    start_st[0] = x_cs[0] + r_min * np.cos(theta_t1)
    start_st[1] = x_cs[1] + r_min * np.sin(theta_t1)
    end_st[0] = x_cg[0] + r_min * np.cos(theta_t2)
    end_st[1] = x_cg[1] + r_min * np.sin(theta_t2)
    dist_st = np.sqrt(np.sum(np.power(end_st - start_st, 2)))

    return np.array([theta_s, delta_theta1]), np.array([theta_t2, delta_theta2]), combined_arc_length, dist_st

def get_params_lsr(r_min, dist, theta, xs, xg, x_cs, x_cg):
    theta_s  = -np.pi/2 + xs[2]
    theta_t1 = -np.arccos(2 * r_min/dist) + theta
    theta_t2 = -np.arccos(2 * r_min/dist) + theta + np.pi
    theta_g  = np.pi/2 + xg[2]

    delta_theta1 = theta_t1 - theta_s
    delta_theta2 = theta_g - theta_t2

    if (delta_theta1 < 0):
        delta_theta1 += 2 * np.pi
    if (delta_theta2 > 0):
        delta_theta2 -= 2 * np.pi

    combined_arc_length = r_min * (delta_theta1 - delta_theta2)

    start_st = np.array([0.0, 0.0])
    end_st = np.array([0.0, 0.0])
    start_st[0] = x_cs[0] + r_min * np.cos(theta_t1)
    start_st[1] = x_cs[1] + r_min * np.sin(theta_t1)
    end_st[0] = x_cg[0] + r_min * np.cos(theta_t2)
    end_st[1] = x_cg[1] + r_min * np.sin(theta_t2)
    dist_st = np.sqrt(np.sum(np.power(end_st - start_st, 2)))

    return np.array([theta_s, delta_theta1]), np.array([theta_t2, delta_theta2]), combined_arc_length, dist_st

def get_params_lsl(r_min, theta, xs, xg):
    theta_s  = -np.pi/2 + xs[2]
    theta_t1 = -np.pi/2 + theta
    theta_t2 = -np.pi/2 + theta
    theta_g  = -np.pi/2 + xg[2]

    delta_theta1 = theta_t1 - theta_s
    delta_theta2 = theta_g - theta_t2

    if (delta_theta1 < 0):
        delta_theta1 += 2 * np.pi
    if (delta_theta2 < 0):
        delta_theta2 += 2 * np.pi

    combined_arc_length = r_min * (delta_theta1 + delta_theta2)

    return np.array([theta_s, delta_theta1]), np.array([theta_t2, delta_theta2]), combined_arc_length

def sample_dubins_path(combined_arc_length, r_min, x_center1, theta1, isright1, x_center2, theta2, isright2):
    step_size = 0.5
    angular_step_size = step_size/r_min

    # Genenerate the first circular arc path
    if (isright1):
        d_theta = np.arange(theta1[0], theta1[0] + theta1[1] - angular_step_size, -angular_step_size)
    else:
        d_theta = np.arange(theta1[0], theta1[0] + theta1[1] + angular_step_size, angular_step_size)

    path_x_cir1 = x_center1[0] + r_min * np.cos(d_theta)
    path_y_cir1 = x_center1[1] + r_min * np.sin(d_theta)

    # Generate the second circular arc
    if (isright2):
        d_theta = np.arange(theta2[0], theta2[0] + theta2[1] - angular_step_size, -angular_step_size)
    else:
        d_theta = np.arange(theta2[0], theta2[0] + theta2[1] + angular_step_size, angular_step_size)

    path_x_cir2 = x_center2[0] + r_min * np.cos(d_theta)
    path_y_cir2 = x_center2[1] + r_min * np.sin(d_theta)

    # Generate the straight section
    delta_x = path_x_cir2[0] - path_x_cir1[-1]
    delta_y = path_y_cir2[0] - path_y_cir1[-1]
    dist = np.sqrt(delta_x**2 + delta_y**2)
    theta = np.arctan2(delta_y, delta_x)
    
    d_dist = np.arange(step_size, dist, step_size)
    path_x_str = path_x_cir1[-1] + d_dist * np.cos(theta)
    path_y_str = path_y_cir1[-1] + d_dist * np.sin(theta)

    path_length = dist + combined_arc_length
    
    # Return concatenated path
    return np.concatenate((path_x_cir1, path_x_str, path_x_cir2)), np.concatenate((path_y_cir1, path_y_str, path_y_cir2)), path_length

def plot_arrow(pose, axes):
    color = 'black'
    length = 1.5

    delta = np.zeros_like(pose[0:2])
    delta[0] = length * np.cos(pose[2])
    delta[1] = length * np.sin(pose[2])

    axes.arrow(pose[0], pose[1], delta[0], delta[1], head_width=0.2, head_length=0.3, color=color)

def plot_arrows(pose, axes):
    color = 'black'
    length = 0.5

    for i in range(0, pose.shape[0]):
        delta = np.zeros_like(pose[i, 0:2])
        delta[0] = length * np.cos(pose[i, 2])
        delta[1] = length * np.sin(pose[i, 2])

        axes.arrow(pose[i, 0], pose[i, 1], delta[0], delta[1], head_width=0.1, head_length=0.15, color=color)

if __name__ == "__main__":
    path_type = sys.argv[1]
    r_min = 4.08106
    x_start = np.array([0.0, 0.0, np.deg2rad(0)])
    x_goal = np.array([20.0, -20.0, np.deg2rad(90)])

    path_x, path_y, center1, center2 = generate_dubins_path(path_type, r_min, x_start, x_goal)

    # Plot the results
    figure, axes = plt.subplots()
    axes.scatter(path_x, path_y, label=f"Dubins {path_type} path Python", color='blue', s=10)
    axes.scatter(path[:, 0], path[:, 1], label=f"Dubins {path_type} path C++", marker='x', color='red', s=10)
    axes.plot(center1[0], center1[1], '.', markersize=8, color='black')
    axes.plot(center2[0], center2[1], '.', markersize=8, color='black')
    circle1 = patch.Circle(center1, r_min, fill = False, color='green', linestyle='-.')
    circle2 = patch.Circle(center2, r_min, fill = False, color='green', linestyle='-.')
    axes.add_artist(circle1)
    axes.add_artist(circle2)
    # plot_arrow(x_start, axes)
    # plot_arrow(x_goal, axes)
    plot_arrows(path, axes)
    axes.axis('equal')
    axes.legend()
    axes.grid(True)

    plt.show()