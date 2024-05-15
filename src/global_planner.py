#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations
from nav_msgs.msg import Odometry
from std_msgs.msg import Char, Float32MultiArray

# Get external parameters from ROS parameter server
pose_topic_name = rospy.get_param("/global_planner/pose_topic_name", "/odometry")


""" Needs to remain stopped until a new waypoint is given using a flag
the flag is to be used to print no feasible path only once """

""" Define the points representing the grid:
- First 16 points are the outside region 
- Last 8 are the inside region """
points = [
    (-20.1, -20.5),
    (-20.1, -2.1),
    (-20.1, 2.2), 
    (-20.1, 20.5),
    (-17.0, 23.75),
    (-2.1, 23.75),
    (2.2, 23.75),
    (17.0, 23.75),
    (20.1, 20.5),
    (20.1, 2.2),
    (20.1, -2.1),
    (20.1, -20.5),
    (17.0, -23.75),
    (2.1, -23.75),
    (-2.2, -23.75),
    (-17.0, -23.75),
    
    (-7.5, -2.1),
    (-7.5, 2.2),
    (-2.1, 8.5),
    (2.2, 8.5),
    (7.5, 2.2),
    (7.5, -2.1),
    (2.1, -8.5),
    (-2.2, -8.5) ]

# Points Dictionary {char: point}
keys = range(ord('A'), ord('A') + len(points))
keys = [chr(key) for key in keys]
point_dict = dict(zip(keys, points))

# Display points dictionary
print("Global Planner - Points Dictionary:")
for key, value in point_dict.items():
    print(key, value) 

""" Neighbors Dictionary {point: [neighboring points]} 
Note: This node graph is directional i.e. nodes a and b can be neighbors while b and a are not """
neighbors = {points[0]: [points[1]],
             points[1]: [points[2], points[16]],
             points[2]: [points[3]],
             points[3]: [points[4]],
             points[4]: [points[5]],
             points[5]: [points[6], points[18]],
             points[6]: [points[7]],
             points[7]: [points[8]],
             points[8]: [points[9]],
             points[9]: [points[10], points[20]],
             points[10]: [points[11]],
             points[11]: [points[12]],
             points[12]: [points[13]],
             points[13]: [points[14], points[22]],
             points[14]: [points[15]],
             points[15]: [points[0]],
             points[16]: [points[21], points[23]],
             points[17]: [points[2], points[23]],
             points[18]: [points[17], points[23]],
             points[19]: [points[6], points[17]],
             points[20]: [points[19], points[17]],
             points[21]: [points[10], points[19]],
             points[22]: [points[21], points[19]],
             points[23]: [points[14], points[21]]}

# Define global variables for subscriber data
current_position = np.array([0,0])
current_heading = np.array(0)

# Define variable to start when simulation is running 
simulation_running = False

# Define a function to calculate the bearing between two points
def calculate_bearing(point1, point2):
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]
    bearing = np.arctan2(delta_y, delta_x)
    return bearing

# Define a function to find the node with the closest bearing to a given heading
def find_closest_node_to_heading(given_point, vehicle_heading):
    bearings = []
    distances = []
    for point in points:
        bearing = calculate_bearing(given_point, point)
        distance = euclidean_distance(given_point, point)
        bearings.append((bearing, point))
        distances.append((distance, point))
    
    bearings.sort(key=lambda x: np.abs(normalize_angle(x[0] - vehicle_heading)))
    
    # Get candidates with bearing within 60 degrees of the minimum bearing difference
    candidates = [x[1] for x in bearings if np.abs(normalize_angle(x[0] - vehicle_heading)) <= np.pi/2]
    # Calculate distances to candidates and filter those with less distance
    distances_to_candidates = [(euclidean_distance(given_point, candidate), candidate) for candidate in candidates]
    # Check if distances_to_candidates is empty
    if not distances_to_candidates:
        return min(bearings, key=lambda x: np.abs(normalize_angle(x[0] - vehicle_heading)))[1]  # No candidates found
    # min_distance = min(distances_to_candidates)[0]
    # closest_nodes = [candidate for distance, candidate in distances_to_candidates if distance <= min_distance]
    
    # Return the closest node among the filtered candidates
    return min(distances_to_candidates, key=lambda x: x[0])[1]

# Define a function to find the edges between given points
def get_edges(points):
    edges = set()
    for i, point1 in enumerate(points):
        x1, y1 = point1
        for j, point2 in enumerate(points):
            if i != j:  # Avoid comparing the point with itself
                x2, y2 = point2
                if x1 == x2 or y1 == y2:
                    edge = ((x1, y1), (x2, y2)) if (x1, y1) < (x2, y2) else ((x2, y2), (x1, y1))
                    edges.add(edge)
    edges = remove_longer_collinear_edges(edges,points)
    return edges

# Define a function to remove edges that aren't neighboring
def remove_longer_collinear_edges(edges, points):
    collinear_edges = set()
    
    # Iterate over each point
    for point in points:
        edges_containing_point = [edge for edge in edges if point in edge]
        
        # Calculate the distances of edges containing the point
        edge_distances = []
        for edge in edges_containing_point:
            dist1 = euclidean_distance(point, edge[0])
            dist2 = euclidean_distance(point, edge[1])
            edge_distances.append(max(dist1, dist2))
        
        if edge_distances:
            # Calculate average distance
            avg_distance = sum(edge_distances) / len(edge_distances)
            
            # Remove edges longer than 5 units from the average distance
            for i, edge_dist in enumerate(edge_distances):
                if edge_dist > avg_distance + 5:
                    collinear_edges.add(edges_containing_point[i])

    return edges - collinear_edges

# Define a function to calculate the Manhattan distance between two points
def manhattan_distance(point1, point2):
    return np.abs(point1[0] - point2[0]) + np.abs(point1[1] - point2[1])

# Define a function to calculate the Euclidean distance between two points
def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Define a function to find the neighbors of a given point in the grid
def get_neighbors(point, edges):
    neighbors = []
    for edge in edges:
        if point in edge:
            neighbor = edge[0] if edge[1] == point else edge[1]
            neighbors.append(neighbor)
    return neighbors

# Define a function to normalize angles to be within [-π, π]
def normalize_angle(angle):
    angle = np.fmod(angle, 2*np.pi)  # Normalize angle to be within [0, 2π]
    
    if angle > np.pi:  # Shift to [-π, π] if necessary
        angle -= 2.0 * np.pi
    elif angle<-np.pi:
        angle+= 2*np.pi    
    return angle

# Define the A* algorithm or A since it uses the manhattan distance
def astar(start, goal):
    open_set = {start}
    came_from = {}
    g_score = {point: float('inf') for point in points}
    g_score[start] = 0
    f_score = {point: float('inf') for point in points}
    f_score[start] = manhattan_distance(start, goal)

    while open_set:
        current = min(open_set, key=lambda point: f_score[point])
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        open_set.remove(current)
        for neighbor in neighbors[current]:
            tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + manhattan_distance(neighbor, goal)
                if neighbor not in open_set:
                    open_set.add(neighbor)

    return None  # No path found

# Setting the map traversal direction
def get_filtered_neighbors(current, heading, points, first_point):
    neighbors = []
    current_x, current_y = current
    for neighbor in get_neighbors(current, get_edges(points)):
        neighbor_x, neighbor_y = neighbor
        if first_point and np.abs(heading - calculate_bearing(current, neighbor)) > 3 * np.pi / 4:
            continue  # Skip this neighbor as it's behind the vehicle
        # Condition to consider only points with a higher y coordinate when x is negative and vice versa
        if current_x < 0 and neighbor_y >= current_y:
            neighbors.append(neighbor)
        elif current_x > 0 and neighbor_y <= current_y:
            neighbors.append(neighbor)
        # Condition for y being negative, x can only be decreasing
        elif current_y < 0 and neighbor_x <= current_x:
            neighbors.append(neighbor)
        elif current_y > 0 and neighbor_x >= current_x:
            neighbors.append(neighbor)
        # Condition for centerpoint then all nodes are to be considered
        elif current_x == 0 and current_y == 0:
            neighbors.append(neighbor)
    return neighbors


# Callback function to handle incoming Odometry messages
def odom_callback(msg:Odometry):
    global current_position, current_heading, simulation_running

    # Update current pose
    current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    orientation = msg.pose.pose.orientation

    # Convert quaternion to Euler angles
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
    
    # Yaw wrt x-axis
    current_heading = normalize_angle(yaw + np.pi/2) 
    
    simulation_running = True


def goal_point_callback(msg:Char):
    rospy.loginfo("New goal point: %c", chr(msg.data))

    # Set goal point
    goal = point_dict.get(chr(msg.data), None)

    # Wait till simulator is up and running
    while not simulation_running:
        pass

    # Find path to goal
    if goal is not None:
        # Find the closest node that is ahead of the vehicle
        closest_node = find_closest_node_to_heading(current_position, current_heading)
        # rospy.loginfo(f"Closest node to given point {current_position} with heading {current_heading}: {closest_node}")

        # Find a path with respect to map direction
        path = astar(closest_node, goal)
        if path is not None:
            # Publish path to simulator and behavioral planner
            # Extract x and y values separately
            x_values = [xy[0] for xy in path]
            y_values = [xy[1] for xy in path]
            
            # Combine x and y values into a single array
            path_data = x_values + y_values
            
            # Create a Float32MultiArray message
            path_msg = Float32MultiArray(data=path_data)

            # Publish full path msg
            path_publisher.publish(path_msg)  
        else:
            rospy.loginfo("No path found from current position: [%.4f, %.4f] to goal point: [%.4f, %.4f]",
                          current_position[0], current_position[1], goal[0], goal[1])
    else:
        rospy.loginfo("Point does not exist")


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('global_planner')
    rospy.sleep(1)

    # Initialize publishers
    path_publisher = rospy.Publisher('/global_planner/path', Float32MultiArray, queue_size=0, latch=True)

    # Initialize subscribers
    odom_sub = rospy.Subscriber(pose_topic_name, Odometry, odom_callback)
    goal_sub = rospy.Subscriber('/global_planner/set_goal', Char, goal_point_callback)

    # Spin forever
    rospy.spin()