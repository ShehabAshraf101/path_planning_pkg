#!/usr/bin/env python3

# Libraries
import numpy as np
from scipy.interpolate import CubicSpline
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# Simple straight path
def path_generator_straight(path_length, step_size, position_current):
    path_x = np.arange(position_current[0], position_current[0] + path_length + step_size, step_size)
    path_y = position_current[1] * np.ones_like(path_x)

    return path_x, path_y


# Circular Path
def path_generator_circular(path_length, path_radius, step_size, position_current, rotate_ccw=True):
    theta_init = np.pi
    center = np.array([position_current[0] - path_radius * np.sin(theta_init), position_current[1] - path_radius * np.cos(theta_init)])
    if (rotate_ccw):
        theta = np.arange(theta_init, theta_init + path_length + step_size, step_size)
    else:
        theta = np.arange(theta_init - path_length, theta_init + step_size, step_size)
        theta = np.flip(theta)
    path_x = center[0] + path_radius * np.sin(theta)
    path_y = -(center[1] + path_radius * np.cos(theta))

    return path_x, path_y


# Straight Path with Lane Change
def path_generator_lane_change(path_length_straight, path_length_curve_x, step_size, lane_width, position_current, turn_left=True):
    # Define the first straight path
    path_x_straight_1, path_y_straight_1 = path_generator_straight(path_length_straight, step_size, position_current)

    # Define the curved path
    path_length_curve = 1.0 * np.sqrt((lane_width**2 + path_length_curve_x**2))
    coeff_x = np.array([0.0, path_length_curve_x/path_length_curve])
    coeff_y = np.array([0.0, 0.0, 3 * lane_width/path_length_curve**2, -2 * lane_width/path_length_curve**3])
    curve_s = np.arange(0.0, path_length_curve + step_size, step_size)
    path_x_curve = coeff_x[0] + coeff_x[1] * curve_s
    path_y_curve = coeff_y[0] + coeff_y[1] * curve_s + coeff_y[2] * curve_s**2 + coeff_y[3] * curve_s**3
    if (not turn_left):
        path_y_curve = -path_y_curve
    path_x_curve += path_x_straight_1[-1]
    path_y_curve += path_y_straight_1[-1]

    # Define the last straight path
    path_x_straight_2, path_y_straight_2 = path_generator_straight(path_length_straight, step_size, np.array([path_x_curve[-1], path_y_curve[-1]]))

    # Remove the last element in the first straight and curved paths to avoid repeated points
    path_x_straight_1 = path_x_straight_1[:-1]
    path_y_straight_1 = path_y_straight_1[:-1]
    path_x_curve = path_x_curve[:-1]
    path_y_curve = path_y_curve[:-1]

    # Return the concatenation of all 3 paths
    return np.concatenate((path_x_straight_1, path_x_curve, path_x_straight_2)), np.concatenate((path_y_straight_1, path_y_curve, path_y_straight_2))


# Infinity Path
def path_generator_infinity(path_radius, step_size, position_current):
    # Calculate the lemniscate constant a
    path_constant = 4 * path_radius/(4 - np.sqrt(6))

    # Generate the infinity sign (Lemniscate of Bernouli)
    t = np.arange(-np.pi/2, 1.5*np.pi + step_size, step_size)
    path = np.array([(path_constant * np.sin(t) * np.cos(t))/(1 + np.sin(t)**2),
                    -(path_constant * np.cos(t))/(1 + np.sin(t)**2)])
    path[0, :] += position_current[0]
    path[1, :] += position_current[1]
    
    # Rotate path to align with orientation of the vehicle
    theta = np.arctan2(path[1, 1] - path[1, 0], path[0, 1] - path[0, 0])
    rot_theta = np.array([[np.cos(theta), np.sin(theta)],
                         [-np.sin(theta), np.cos(theta)]]
                        )
    path = np.matmul(rot_theta, path)

    return path[0, :], path[1, :]


# Euclidean distance 
def euclidean_distance(x, y):
    """
    Calculate the accumulated Euclidean distances for each point in the arrays x and y.

    Parameters:
    - x: numpy array or list, x-coordinates of the points
    - y: numpy array or list, y-coordinates of the points

    Returns:
    - accumulated_distances: numpy array containing the accumulated Euclidean distances
    """
    points = np.column_stack((x, y))
    pairwise_distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    accumulated_distances = np.cumsum(pairwise_distances)
    return np.insert(accumulated_distances, 0, 0)  # Insert 0 at the beginning for the starting point


# Path Selection
def generate_path(path_type, *args):
    """
    Generate a path based on the specified path type.

    Parameters:
    - path_type: str, Type of path ('straight', 'circular', 'lane_change', 'infinity')
    - *args: Additional arguments required for each path type

    Returns:
    - Tuple of (path_x, path_y)
    """
    if path_type == 'straight':
        path_length, step_size, position_current = args
        return path_generator_straight(path_length, step_size, position_current)
    
    elif path_type == 'circular':
        path_length, path_radius, step_size, position_current, rotate_ccw = args
        return path_generator_circular(path_length, path_radius, step_size, position_current, rotate_ccw=rotate_ccw)
    
    elif path_type == 'lane_change':
        path_length_straight, path_length_curve_x, step_size, lane_width, position_current, turn_left = args
        return path_generator_lane_change(path_length_straight, path_length_curve_x, step_size, lane_width, position_current, turn_left)
    
    elif path_type == 'infinity':
        path_radius, step_size, position_current = args
        return path_generator_infinity(path_radius, step_size, position_current)
    
    else:
        raise ValueError("Invalid path type. Choose from 'straight', 'circular', 'lane_change', 'infinity'.")

def path_data(path_type, *args):
    # Generate path based on input parameters
    path_x, path_y = generate_path(path_type,*args)

    # Fit cubic splines to both x and y
    accumulated_euclid_dist = euclidean_distance(path_x, path_y)
    spline_x = CubicSpline(accumulated_euclid_dist, path_x, bc_type='natural')
    spline_y = CubicSpline(accumulated_euclid_dist, path_y, bc_type='natural')

    # Estimate path derivatives and curvature using the fit splines
    x_dot = spline_x.derivative()
    y_dot = spline_y.derivative()
    x_ddot = x_dot.derivative()
    y_ddot = y_dot.derivative()
    curvature = (-x_ddot(accumulated_euclid_dist) * y_dot(accumulated_euclid_dist) + \
                 x_dot(accumulated_euclid_dist) * y_ddot(accumulated_euclid_dist))/ \
                ((x_dot(accumulated_euclid_dist))**2 + (y_dot(accumulated_euclid_dist))**2)**(1.5)
    # heading = np.arctan2(y_dot(accumulated_euclid_dist), x_dot(accumulated_euclid_dist))

    # Generate velocity profile
    velocity_min = 2.0
    velocity_max = 8.0
    velocity = np.maximum(velocity_max - 10.0 * np.sqrt(np.abs(curvature)), velocity_min)
    velocity_step = 0.5
    velocity_ramp = np.arange(0.0, velocity_max, velocity_step)
    velocity[:velocity_ramp.shape[0]] = np.minimum(velocity_ramp, velocity[:velocity_ramp.shape[0]])
    velocity[-velocity_ramp.shape[0]:] = np.minimum(np.flip(velocity_ramp), velocity[-velocity_ramp.shape[0]:])

    # Return concatenated trajectory
    return np.concatenate((path_x, path_y, curvature, velocity))

def main():
    # Paths (one is to be uncommented)
    # path_length = 75.0
    # step_size = 0.05
    # position_current = np.array([0.0, 0.0])
    # traj_concat = path_data('straight',path_length, step_size, position_current)

    # path_length = 2 * np.pi
    # path_radius = 6
    # step_size = 0.5 * np.pi/180
    # position_current = np.array([0.0, 0.0])
    # traj_concat = path_data('circular', path_length, path_radius, step_size, position_current, False)
    
    # path_length_straight = 75.0
    # path_length_curve_x = 30.0
    # step_size = 0.05
    # lane_width = 3.7
    # position_current = np.array([-90.0, 0.0])
    # traj_concat = path_data('lane_change', path_length_straight, path_length_curve_x, step_size, lane_width, position_current, True)
    
    path_radius = 7.0
    step_size = 0.5 * np.pi/180
    position_current = np.array([0.0, 0.0])
    traj_concat = path_data('infinity', path_radius, step_size, position_current)
    
    rospy.init_node('path_planner')
    pub_traj = rospy.Publisher('/path_planner/trajectory', Float32MultiArray, queue_size=0, latch=True)
    rate = rospy.Rate(1)
    initial_delay = 2
    rospy.sleep(initial_delay)
    has_published_traj = False
    while not rospy.is_shutdown():

        if ((not has_published_traj) and (pub_traj.get_num_connections() > 0)):
            msg = Float32MultiArray()
            msg.data = traj_concat.tolist()
            msg.layout.data_offset = 0 

            # create two dimensions in the dim array
            msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

            # dim[0] is the horizontal dimension of your matrix
            msg.layout.dim[0].label = "samples"
            msg.layout.dim[0].size = int(traj_concat.shape[0]/4)
            msg.layout.dim[0].stride = traj_concat.shape[0]

            # dim[1] is the vertical dimension of your matrix
            msg.layout.dim[1].label = "variables(X,Y,Curvature,Velocity)"
            msg.layout.dim[1].size = 4
            msg.layout.dim[1].stride = 4
            
            pub_traj.publish(msg)
            has_published_traj = True
        
        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
