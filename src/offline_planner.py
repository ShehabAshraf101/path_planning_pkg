#!/usr/bin/env python3

### Libraries ###
import numpy as np
from scipy.interpolate import CubicSpline
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

### Loading Parameters from the ROSPARAM Server ###

# Path Parameters
path_type               = rospy.get_param("/offline_planner/path_type")
path_length             = rospy.get_param("/offline_planner/path_length")
path_length_straight    = rospy.get_param("/offline_planner/path_length_straight")
path_length_curve_x     = rospy.get_param("/offline_planner/path_length_curve_x")
path_radius             = rospy.get_param("/offline_planner/path_radius")
step_size               = rospy.get_param("/offline_planner/step_size")
rotate_ccw              = rospy.get_param("/offline_planner/rotate_ccw")
lane_width              = rospy.get_param("/offline_planner/lane_width")
turn_left               = rospy.get_param("/offline_planner/turn_left")
position_current        = eval(rospy.get_param("/offline_planner/position_current"))
publish_heading         = rospy.get_param("/offline_planner/publish_heading")

# Velocity Profile Parameters
velocity_max = rospy.get_param('/offline_planner/velocity_max', default=8.0)
acc_long_max = rospy.get_param('/offline_planner/acc_long_max', default=2.0)
dec_long_max = rospy.get_param('/offline_planner/dec_long_max', default=3.0)
acc_lat_max  = rospy.get_param('/offline_planner/acc_lat_max', default=2.5)


### Path Generation ### 

# Straight Path 
def path_generator_straight(path_length, step_size, position_current):
    path_x = np.arange(position_current[0], position_current[0] + path_length + step_size, step_size)
    path_y = position_current[1] * np.ones_like(path_x)

    return path_x, path_y

# Ciruclar Path
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

    # Generate the infinity sign (Lemniscate of Bernoulli)
    t = np.arange(-np.pi/2, 1.5*np.pi + step_size, step_size)
    path = np.array([(path_constant * np.sin(t) * np.cos(t))/(1 + np.sin(t)**2),
                    -(path_constant * np.cos(t))/(1 + np.sin(t)**2)])
    
    # Rotate path to align with orientation of the vehicle
    theta = np.arctan2(path[1, 1] - path[1, 0], path[0, 1] - path[0, 0])
    rot_theta = np.array([[np.cos(theta), np.sin(theta)],
                         [-np.sin(theta), np.cos(theta)]]
                        )
    path = np.matmul(rot_theta, path)

    path[0, :] += position_current[0]
    path[1, :] += position_current[1]

    return path[0, :], path[1, :]

# Path Selection
def path_generator(path_type, *args):
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


### Heading and Curvature Estimation ### 

# Calculate Accumulated Euclidean Distance
def calc_euclidean_distance(path_x, path_y):
    """
    Calculate the accumulated Euclidean distances for each point in the arrays x and y.

    Parameters:
    - path_x: numpy array, x-coordinates of the points
    - path_y: numpy array, y-coordinates of the points

    Returns:
    - pairwise_distances: numpy array containing the Euclidean distances between consecutive points
    - accumulated_distances: numpy array containing the accumulated Euclidean distances
    """
    pairwise_distances = np.empty_like(path_x)
    pairwise_distances[0] = 0.0
    pairwise_distances[1:] = np.hypot(np.diff(path_x), np.diff(path_y))
    accumulated_distances = np.cumsum(pairwise_distances)

    # Insert 0 at the beginning for the starting point
    return pairwise_distances, accumulated_distances

# Calculate Path Heading and Curvature
def calc_heading_curvature(path_x, path_y, euclid_dist):
    # Fit cubic splines to both x and y
    spline_x = CubicSpline(euclid_dist, path_x, bc_type='natural')
    spline_y = CubicSpline(euclid_dist, path_y, bc_type='natural')

    # Get estimate of path derivatives using the fit splines
    x_dot = spline_x.derivative()
    y_dot = spline_y.derivative()
    x_ddot = x_dot.derivative()
    y_ddot = y_dot.derivative()

    # Calculate and return heading and curvature estimates using path derivatives
    heading = np.arctan2(y_dot(euclid_dist), x_dot(euclid_dist))
    curvature = (-x_ddot(euclid_dist) * y_dot(euclid_dist) + x_dot(euclid_dist) * y_ddot(euclid_dist))/ \
                ((x_dot(euclid_dist))**2 + (y_dot(euclid_dist))**2)**(1.5)
    
    return heading, curvature


### Velocity Profile Generation ###

# Generate Velocity Profile using Simplified Forward-Backward Solver
def velocity_profile_generator(pairwise_dist, curvature):
    # Generate initial profile based on curvature and max lateral acceleration
    curvature_abs = np.abs(curvature)
    velocity = np.where(curvature_abs == 0, velocity_max, np.sqrt(acc_lat_max/curvature_abs))
    velocity = np.where(velocity > velocity_max, velocity_max, velocity)
    velocity[0] = 0.1
    velocity[-1] = 0.0

    # Apply forward pass on velocity profile (have to loop over array)
    for i in range(0, velocity.shape[0] - 1):
        velocity_squared = velocity[i] * velocity[i]
        acc_lat = velocity_squared * curvature_abs[i]
        acc_long = acc_long_max * np.sqrt(np.maximum(1.0 - (acc_lat * acc_lat)/(acc_lat_max * acc_lat_max), 0.0))
        velocity_new = np.sqrt(velocity_squared + 2.0 * acc_long * pairwise_dist[i+1])
        velocity[i+1] = velocity_new if (velocity_new < velocity[i+1]) else velocity[i+1] 

    # Apply backward pass on velocity profile (have to loop over array)
    for i in range(velocity.shape[0] - 1, 0, -1):
        velocity_squared = velocity[i] * velocity[i]
        acc_lat = velocity_squared * curvature_abs[i]
        acc_long = dec_long_max * np.sqrt(np.maximum(1.0 - (acc_lat * acc_lat)/(acc_lat_max * acc_lat_max), 0.0))
        velocity_new = np.sqrt(velocity_squared + 2.0 * acc_long * pairwise_dist[i])
        velocity[i-1] = velocity_new if (velocity_new < velocity[i-1]) else velocity[i-1]

    return velocity

### Full Trajectory Generation ###

# Generate Trajectory using the Previous Functions
def traj_generator(path_type, *args):
    # Generate path based on input parameters
    path_x, path_y = path_generator(path_type,*args)

    # Get pairwise and accumulated Euclidean distances as well as heading and curvature
    pairwise_dist, accumulated_euclid_dist = calc_euclidean_distance(path_x, path_y)
    heading, curvature = calc_heading_curvature(path_x, path_y, accumulated_euclid_dist)

    # Generate velocity profile
    velocity = velocity_profile_generator(pairwise_dist, curvature)

    # Return concatenated trajectory (return heading if closed_loop and curvature if open_loop)
    if publish_heading:
        return np.concatenate((path_x, path_y, heading, velocity)) 
    else:
        return np.concatenate((path_x, path_y, curvature, velocity))

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('offline_planner')

    # Initialize publishers 
    pub_traj = rospy.Publisher('/offline_planner/trajectory', Float32MultiArray, queue_size=0, latch=True)

    # Report the values of all retreived parameters
    rospy.loginfo("path_type: %s", path_type)
    rospy.loginfo("path_length: %.4f", path_length)
    rospy.loginfo("path_length_straight: %.4f", path_length_straight)
    rospy.loginfo("path_length_curve_x: %.4f", path_length_curve_x)
    rospy.loginfo("path_radius: %.4f", path_radius)
    rospy.loginfo("step_size: %.4f", step_size)
    rospy.loginfo("rotate_ccw: %s", "True" if rotate_ccw==True else "False")
    rospy.loginfo("lane_width: %.4f", lane_width)
    rospy.loginfo("turn_left: %s", "True" if turn_left==True else "False")
    rospy.loginfo("position_current: [%.4f, %.4f]", position_current[0], position_current[1])
    rospy.loginfo("publish_heading: %s", "True" if publish_heading==True else "False")
    rospy.loginfo("\n")

    rospy.loginfo("velocity_max: %.4f", velocity_max)
    rospy.loginfo("acc_long_max: %.4f", acc_long_max)
    rospy.loginfo("acc_lat_max: %.4f", acc_lat_max)
    rospy.loginfo("dec_long_max: %.4f", dec_long_max)
    rospy.loginfo("\n\n")
    # Initialize path (to be modified)

    if path_type == "straight":
            traj_concat = traj_generator("straight", path_length_straight, step_size, position_current)
    elif path_type == "circular":
            traj_concat = traj_generator("circular", path_length, path_radius, step_size, position_current, rotate_ccw)
    elif path_type == "lane_change":
            traj_concat = traj_generator("lane_change", path_length_straight, path_length_curve_x, step_size, lane_width, position_current, turn_left)
    elif path_type == "infinity":
            traj_concat = traj_generator("infinity", path_radius, step_size, position_current)
    else:
            rospy.logwarn("Invalid path_type. Choosing default: 'straight'")
            traj_concat = traj_generator("straight", path_length, step_size, position_current)

    # Main infinite loop
    rate = rospy.Rate(1)
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