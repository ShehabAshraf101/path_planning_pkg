#!/usr/bin/env python3

# Libraries
import numpy as np
from scipy.interpolate import CubicSpline
import rospy
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
# Simple straight path
def path_generator_straight(path_length, step_size, position_current):
    path_x = np.arange(position_current[0], position_current[0] + path_length + step_size, step_size)
    path_y = position_current[1] * np.ones_like(path_x)

    return path_x, path_y


# Circular Path
def path_generator_circular(path_length, path_radius, step_size, position_current, rotate_ccw=True):
    theta_init = np.pi
    center = np.array([position_current[0] - path_radius * np.cos(theta_init), position_current[1] - path_radius * np.sin(theta_init)])
    if (rotate_ccw):
        theta = np.arange(theta_init, theta_init + path_length + step_size, step_size)
    else:
        theta = np.arange(theta_init - path_length, theta_init + step_size, step_size)
        theta = np.flip(theta)
    path_x = center[0] + path_radius * np.cos(theta)
    path_y = center[1] + path_radius * np.sin(theta)

    return path_x, path_y


# Straight Path with Lane Change
def path_generator_lane_change(path_length_straight, path_length_curve_x, step_size, lane_width, position_current, turn_left=True):
    # Define the first straight path
    path_x_straight_1, path_y_straight_1 = path_generator_straight(path_length_straight, step_size, position_current)

    # Define the curved path
    path_length_curve = 0.8 * np.sqrt((lane_width**2 + path_length_curve_x**2))
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
    path_x = (path_constant * np.cos(t))/(1 + np.sin(t)**2)
    path_y = (path_constant * np.sin(t) * np.cos(t))/(1 + np.sin(t)**2)
    path_x += position_current[0]
    path_y += position_current[1]

    return path_x, path_y


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
        return path_generator_circular(path_length, path_radius, step_size, position_current, rotate_ccw)
    
    elif path_type == 'lane_change':
        path_length_straight, path_length_curve_x, step_size, lane_width, position_current, turn_left = args
        return path_generator_lane_change(path_length_straight, path_length_curve_x, step_size, lane_width, position_current, turn_left)
    
    elif path_type == 'infinity':
        path_radius, step_size, position_current = args
        return path_generator_infinity(path_radius, step_size, position_current)
    
    else:
        raise ValueError("Invalid path type. Choose from 'straight', 'circular', 'lane_change', 'infinity'.")

def path_data(path_type, *args):
    path = generate_path(path_type,*args)
    E = euclidean_distance(path[0],path[1])
    X = CubicSpline(E,path[0],bc_type='natural')
    Y = CubicSpline(E,path[1],bc_type='natural')
    X_dot = X.derivative()
    Y_dot = Y.derivative()
    X_ddot = X_dot.derivative()
    Y_ddot = Y_dot.derivative()
    velocity = np.full([76],10.0)
    curvature = (-X_ddot(E) * Y_dot(E) + X_dot(E) * Y_ddot(E))/ ((X_dot(E))**2 + (Y_dot(E))**2)**(3/2)
    heading = np.arctan2(Y_dot(E),X_dot(E))
    A = path[0],path[1],velocity,curvature,heading
    O = np.array([A[0],A[1],A[2],A[3],A[4]], dtype=np.float32)
    return O

def main():
    # Path data
    path_length = path_length_straight = 75.0
    path_radius = 6
    path_length_curve_x = 500
    step_size = 1.0
    lane_width = 3.7
    position_current = np.array([1.0, 3.0])

    # Paths (one is to be uncommented)
    A = path_data('straight',path_length, step_size, position_current)

    # A = path_data('circular',path_length, path_radius, step_size, position_current, False)
    
    # A = path_data('lane_change',path_length_straight, path_length_curve_x, step_size, lane_width, position_current, True)
    
    path_radius = 7.0
    # A = path_data('infinity',path_radius, step_size, position_current)
    
    rospy.init_node('path_planner')
    pub = rospy.Publisher('path_planner', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = Float32MultiArray()
        msg.data = (A.reshape([5*76]).tolist())
        msg.layout.data_offset = 0 

        # create two dimensions in the dim array
        msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

        # dim[0] is the vertical dimension of your matrix
        msg.layout.dim[0].label = "variables(X,Y,Velocity,Curvature,Heading)"
        msg.layout.dim[0].size = 5
        msg.layout.dim[0].stride = 5*76
        # dim[1] is the horizontal dimension of your matrix
        msg.layout.dim[1].label = "samples"
        msg.layout.dim[1].size = 76
        msg.layout.dim[1].stride = 76
        
        pub.publish(msg)  
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
