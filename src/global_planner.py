#!/usr/bin/env python3

import rospy
import math
import numpy as np
import tf.transformations
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from path_planning_pkg.msg import Waypoint

""" Needs to remain stopped until a new waypoint is given using a flag
the flag is to be used to print no feasible path only once """

# Define the 9 points representing the grid
points = [
    (21.9125, 25.7625), (21.9125, 0), (21.9125, -25.725),
    (0, 25.7625), (0, 0), (0, -25.725),
    (-22.05, 25.7625), (-22.05, 0), (-22.05, -25.725) ]

# Dictionary
dict = {'A': (21.9125, 25.7625), 'B': (21.9125, 0), 'C': (21.9125, -25.725), 
 'D': (0, 25.7625), 'E': (0, 0), 'F': (0, -25.725), 
 'G': (-22.05, 25.7625), 'H': (-22.05, 0), 'I': (-22.05, -25.725)}

# Define global variables for subscriber data
current_position = np.array([0,0])
current_velocity = np.array([0,0])
current_heading = np.array(0)

# Define publishing proximity threshold 
proximity_threshold = 5

# Define variable to start when simulation is running 
simulation_running = False

# Define a function to calculate the bearing between two points
def calculate_bearing(point1, point2):
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]
    bearing = math.atan2(delta_y, delta_x)
    return bearing

# Define a function to calculate the distance between two points
def calculate_distance(point1, point2):
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]
    distance = math.sqrt(delta_x**2 + delta_y**2)
    return distance

# Define a function to find the node with the closest bearing to a given heading
def find_closest_node_to_heading(given_point, vehicle_heading):
    bearings = []
    distances = []
    for point in points:
        bearing = calculate_bearing(given_point, point)
        distance = calculate_distance(given_point, point)
        bearings.append((bearing, point))
        distances.append((distance, point))
    
    bearings.sort(key=lambda x: abs(x[0] - vehicle_heading))
    
    # Get candidates with bearing within 45 degrees of the minimum bearing difference
    candidates = [x[1] for x in bearings if abs(x[0] - vehicle_heading) <= math.pi/3]
    # Calculate distances to candidates and filter those with less distance
    distances_to_candidates = [(calculate_distance(given_point, candidate), candidate) for candidate in candidates]
    # Check if distances_to_candidates is empty
    if not distances_to_candidates:
        return min(bearings, key=lambda x: abs(x[0] - vehicle_heading))[1]  # No candidates found
    min_distance = min(distances_to_candidates)[0]
    closest_nodes = [candidate for distance, candidate in distances_to_candidates if distance <= min_distance]
    
    # Return the closest node among the filtered candidates
    return min(closest_nodes, key=lambda x: calculate_distance(given_point, x))

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

# Define a function to find the distance between given points
def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

# Define a function to remove edges that aren't neighboring
def remove_longer_collinear_edges(edges, points):
    collinear_edges = set()
    
    # Iterate over each point
    for point in points:
        edges_containing_point = [edge for edge in edges if point in edge]
        
        # Calculate the distances of edges containing the point
        edge_distances = []
        for edge in edges_containing_point:
            dist1 = distance(point, edge[0])
            dist2 = distance(point, edge[1])
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
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

# Define a function to calculate the Euclidean distance between two points
def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Define a function to find the neighbors of a given point in the grid
def get_neighbors(point, edges):
    neighbors = []
    for edge in edges:
        if point in edge:
            neighbor = edge[0] if edge[1] == point else edge[1]
            neighbors.append(neighbor)
    return neighbors

# Define the A* algorithm or A since it uses the manhattan distance
def astar(start, heading, goal):
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
        for neighbor in get_filtered_neighbors(current, heading, points, current == start):
            tentative_g_score = g_score[current] + 1  # Assuming uniform cost for each step
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
        if first_point and abs(heading - calculate_bearing(current, neighbor)) > 3 * math.pi / 4:
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
def odom_callback(msg):
    global current_position, current_velocity, current_heading, simulation_running
    current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    current_velocity = msg.twist.twist.linear
    orientation = msg.pose.pose.orientation

    # Convert quaternion to Euler angles
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    
    # Yaw wrt x-axis
    current_heading = yaw + math.pi/2 
    if current_heading > math.pi:
        current_heading = math.pi/2 - yaw
    
    simulation_running = True


#Define the main method
def main():
    while not rospy.is_shutdown():

        # Initialize the ROS node
        rospy.init_node('global_planner', anonymous=True)
        rate = rospy.Rate(1)
        rate.sleep()
        # Initialize a publisher for the global_plan topic
        pub = rospy.Publisher('/global_plan', Waypoint, queue_size=10)

        # Odometry subscriber
        rospy.Subscriber('/odom', Odometry, odom_callback)

        # Define current vehicle location and heading
        while not simulation_running:
            pass
        given_point = current_position
        vehicle_heading = current_heading  
        # Find the closest node to the current heading
        closest_node = find_closest_node_to_heading(given_point, vehicle_heading)
        # rospy.loginfo(f"Closest node to given point {given_point} with heading {vehicle_heading}: {closest_node}")
        
        # Define goal point
        goal = dict[rospy.get_param('/global_planner/goal_point')]

        # Find a path with respect to map direction
        path = astar(closest_node, vehicle_heading, goal)
        if path:
        # Path on simulator publisher 
            path_publisher = rospy.Publisher('/path', Float32MultiArray, queue_size=10)
            # Extract x and y values separately
            x_values = [xy[0] for xy in path]
            y_values = [xy[1] for xy in path]
            
            # Combine x and y values into a single array
            path_data = x_values + y_values
            
            # Create a Float32MultiArray message
            path_msg = Float32MultiArray(data=path_data)

        # Local and behavioral planner publisher
            # Create a Waypoint message
            waypoint_msg = Waypoint()
            waypoint_msg.Header.frame_id = "map"  # Assuming the coordinates are in the "map" frame
           
            # Publish path points sequentially
            for i, point in enumerate(path):
                # Change in xy since simulator heading is set incorrectly
                waypoint_msg.pose.position.x = point[1]
                waypoint_msg.pose.position.y = -point[0]
                waypoint_msg.stop_at_waypoint = False

                # Check if final waypoint
                if i == len(path) - 1:
                    waypoint_msg.stop_at_waypoint = True
                else:
                    next_point = path[i + 1]
                    next_heading = calculate_bearing(point, next_point)
                
                # Calculate quaternion for publishing desired heading 
                quaternion  = tf.transformations.quaternion_from_euler(0, 0, next_heading)
                waypoint_msg.pose.orientation.x = quaternion[0]
                waypoint_msg.pose.orientation.y = quaternion[1]
                waypoint_msg.pose.orientation.z = quaternion[2]
                waypoint_msg.pose.orientation.w = quaternion[3]
                while euclidean_distance(point, current_position) >= proximity_threshold and not rospy.is_shutdown():
                    # Keep publishing upcoming waypoint & simulator path
                    path_publisher.publish(path_msg)  
                    pub.publish(waypoint_msg)
                    rate.sleep()  # Wait between publishing

        else:
            rospy.loginfo(f"Couldn't find a path from point: {given_point} to goal: {goal}")

      
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass