#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from path_planning_pkg.msg import Waypoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Define enum for different states
class State:
    DRIVE = 0
    DECELERATE = 1
    STOP = 2

# Define a function to calculate the bearing between two points
def calculate_bearing(point1, point2):
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]
    bearing = np.arctan2(delta_y, delta_x)
    return bearing

# Define a function to calculate the Euclidean distance between two points
def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Define a function to get midpoint between two points
def get_midpoint(point1, point2):
    return np.array([(point1[0] + point2[0])/2, (point1[1] + point2[1])/2])

def normalize_angle(angle):
    angle = np.fmod(angle, 2*np.pi)  # Normalize angle to be within [0, 2π]
    
    if angle > np.pi:  # Shift to [-π, π] if necessary
        angle -= 2.0 * np.pi
    elif angle<-np.pi:
        angle+= 2*np.pi    
    return angle

def average_heading(angle1, angle2):
    # Convert angles to Cartesian coordinates
    x1, y1 = np.cos(angle1), np.sin(angle1)
    x2, y2 = np.cos(angle2), np.sin(angle2)
    
    # Compute the weighted average of the Cartesian coordinates
    avg_x = (x1 + x2) / 2
    avg_y = (y1 + y2) / 2
    
    # Convert the average Cartesian coordinates back to an angle
    avg_angle = np.arctan2(avg_y, avg_x)
    
    return avg_angle

# Define a function to get the closest point on a line to another point
# returns two values: the closest point and boolean whether that point is within the line or outside of it 
def get_closest_to_point(line, point):
    start_point = line[0]
    end_point = line[1]
    start_to_end = start_point - end_point

    t = np.dot(point - start_point, start_to_end)/np.dot(start_to_end, start_to_end)
    closest_point = start_point + t * start_to_end
    if (t < -1.0) or (t > 0.0):
        within_line = False
    else:
        within_line = True
    
    return closest_point, within_line

class BehavioralPlanner:
    def __init__(self):
        rospy.init_node('behavioral_planner')

        # Get external parameters from ROS parameter server
        pose_topic_name = rospy.get_param("/behavioral_planner/pose_topic_name", "/odometry")
        object_topic_name = rospy.get_param("/behavioral_planner/object_topic_name", "/objects")
        global_plan_topic_name = rospy.get_param("/behavioral_planner/global_plan_topic_name", "/global_plan")
        update_rate = rospy.get_param("/behavioral_planner/update_rate", 10.0)
        wp_proximity_up_threshold = rospy.get_param("/behavioral_planner/wp_proximity_up_threshold", 20.0)
        wp_proximity_lw_threshold = rospy.get_param("/behavioral_planner/wp_proximity_lw_threshold", 5.0)
        velocity_threshold = rospy.get_param("/behavioral_planner/velocity_threshold", 0.1)
        stop_sign_time_threshold = rospy.get_param("/behavioral_planner/stop_sign_time_threshold", 5.0)
        yield_sign_time_threshold = rospy.get_param("/behavioral_planner/yield_sign_time_threshold", 2.0)

        # Initialize clas members
        self.rate = rospy.Rate(update_rate)  # 10 Hz
        self.current_state = State.STOP  # Initialize the state to STOP
        self.current_position = np.zeros((2, ))
        self.current_heading = 0.0
        self.current_velocity = 0.0
        self.velocity_threshold = velocity_threshold
        self.stop_start_time = None  # Initialize stop state start time
        self.stop_sign_time_threshold = stop_sign_time_threshold
        self.yield_sign_time_threshold = yield_sign_time_threshold
        self.simulation_running = False
        self.wp_proximity_up_threshold = wp_proximity_up_threshold
        self.wp_proximity_lw_threshold = wp_proximity_lw_threshold
        self.global_plan = None
        self.waypoint_old = np.array([np.inf, np.inf])
        self.waypoint_index = 0

        # Initialize Detection flags 
        self.stop_sign = False
        self.yield_sign = False
        self.cross_walk = False
        self.pedestrian_on_road = False
        self.final_waypoint = False

        # Publisher for waypoints
        self.waypoint_pub = rospy.Publisher('/behavioral_planner/waypoint', Waypoint, queue_size=0, latch=True)

        # Odometry subscriber
        rospy.Subscriber(pose_topic_name, Odometry, self.odom_callback)

        # Subscriber for object detection
        # rospy.Subscriber(object_topic_name, Image, self.object_detection_callback)

        # Subscriber for global plan
        rospy.Subscriber(global_plan_topic_name, Float32MultiArray, self.global_plan_callback)


    def run(self):
        while not rospy.is_shutdown():
            while not self.simulation_running:
                pass
            self.execute_state()
            self.update_state()
            if self.global_plan is not None:
                self.update_waypoint()
            self.rate.sleep()

    def update_state(self):
        if self.current_state == State.DRIVE:
            if self.stop_sign or (self.cross_walk and self.pedestrian_on_road) or self.yield_sign or self.final_waypoint:
                self.current_state = State.DECELERATE
        elif self.current_state == State.DECELERATE:
            if self.current_velocity <= self.velocity_threshold:
                self.current_state = State.STOP
        elif self.current_state == State.STOP:
            if self.stop_start_time is None:
                self.stop_start_time = rospy.Time.now()  # Start counting time when entering stop state
            elapsed_time = (rospy.Time.now() - self.stop_start_time).to_sec()  # Calculate elapsed time
            if (self.cross_walk and not self.pedestrian_on_road) or \
                (self.stop_sign and elapsed_time > self.stop_sign_time_threshold) or \
                (self.yield_sign and elapsed_time > self.yield_sign_time_threshold) or \
                (not self.stop_sign and not self.cross_walk and not self.yield_sign):

                self.current_state = State.DRIVE if not self.final_waypoint else State.STOP
                self.stop_start_time = None  # Reset stop state start time

    def execute_state(self):
        # Execute behavior based on current state
        if self.current_state == State.DRIVE:
            pass
            # rospy.loginfo("Driving...")
        elif self.current_state == State.DECELERATE:
            rospy.loginfo("Decelerating...")
        elif self.current_state == State.STOP:
            pass
            #rospy.loginfo("Stopped...")

    # Function to get and publish the next waypoint
    def update_waypoint(self):
        # Create a Waypoint message
        waypoint_msg = Waypoint()
        waypoint_msg.Header.frame_id = "map"  # Assuming the coordinates are in the "map" frame
        
        waypoint = self.global_plan[self.waypoint_index]
        dist_to_waypoint = euclidean_distance(self.current_position, waypoint)
        angle_to_waypoint = normalize_angle(calculate_bearing(self.current_position, waypoint))
        angle_to_waypoint = np.abs(normalize_angle(self.current_heading - angle_to_waypoint))
        
        # Move to next waypoint if distance is less than the set lower threshold
        if (self.waypoint_index != (self.global_plan.shape[0] - 1)) and \
            ((angle_to_waypoint > np.pi/2) or (dist_to_waypoint < self.wp_proximity_lw_threshold)):
            self.waypoint_index += 1
            waypoint = self.global_plan[self.waypoint_index]
            dist_to_waypoint = euclidean_distance(self.current_position, waypoint)
        
        # Find midpoint within thresholds if distance is greater than set higher threshold
        midpoint = waypoint
        dist_to_midpoint = dist_to_waypoint
        prev_waypoint = self.global_plan[self.waypoint_index - 1] if self.waypoint_index != 0 else self.current_position
        while dist_to_midpoint > self.wp_proximity_up_threshold:
            midpoint = get_midpoint(prev_waypoint, waypoint)
            dist_to_midpoint = euclidean_distance(self.current_position, midpoint)
            if dist_to_midpoint < self.wp_proximity_lw_threshold:
                prev_waypoint = midpoint
            else:
                waypoint = midpoint

        # Publish waypoint message if a new waypoint is available
        if not np.array_equal(self.waypoint_old, midpoint):
            print("Waypoint: ", midpoint)
            waypoint_msg.pose.position.x = midpoint[1] # Change in xy since simulator heading is set incorrectly
            waypoint_msg.pose.position.y = -midpoint[0]
            waypoint_msg.stop_at_waypoint = True if np.array_equal(midpoint, self.global_plan[-1]) else False 
            
            if self.waypoint_index == (self.global_plan.shape[0] - 1):
                next_heading = calculate_bearing(self.global_plan[self.waypoint_index - 1], self.global_plan[self.waypoint_index])
            elif np.array_equal(midpoint, self.global_plan[self.waypoint_index]):
                heading_prev = calculate_bearing(prev_waypoint, midpoint)
                next_heading = average_heading(calculate_bearing(midpoint, self.global_plan[self.waypoint_index + 1]), heading_prev)
            else:
                heading_prev = calculate_bearing(prev_waypoint, midpoint)
                next_heading = average_heading(calculate_bearing(midpoint, self.global_plan[self.waypoint_index]), heading_prev)
            next_heading = normalize_angle(next_heading - np.pi/2) 

            # print(np.rad2deg(next_heading))
            quaternion = quaternion_from_euler(0, 0, next_heading)
            waypoint_msg.pose.orientation.x = quaternion[0]
            waypoint_msg.pose.orientation.y = quaternion[1]
            waypoint_msg.pose.orientation.z = quaternion[2]
            waypoint_msg.pose.orientation.w = quaternion[3]

            self.waypoint_pub.publish(waypoint_msg)
        
        self.waypoint_old = midpoint


    def object_detection_callback(self, msg):
        # Process the object detection data received in 'data' message and update the variables accordingly
        # Example:
        # if data.stop_sign_detected:
        #     self.stop_sign = True
        # if data.cross_walk_detected:
        #     self.cross_walk = True
        # ...
        pass
    
    def global_plan_callback(self, msg:Float32MultiArray):
        # Get the points' x and y coordinates
        num_points = len(msg.data)//2
        points_x = msg.data[:num_points]
        points_y = msg.data[num_points:]

        # Combine the points into a 2D numpy array
        self.global_plan = np.array([[x, y] for x, y in zip(points_x, points_y)])
        
        # Reset closest point to initial point
        self.waypoint_index = 0

    # Callback function to handle incoming Odometry messages
    def odom_callback(self, msg:Odometry):
        # Update current position and velocity
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])    
        self.current_velocity = np.sqrt((msg.twist.twist.linear.x)**2 + (msg.twist.twist.linear.y)**2)
        
        orientation = msg.pose.pose.orientation

        # Convert quaternion to Euler angles
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # Yaw wrt x-axis
        self.current_heading = normalize_angle(yaw + np.pi/2) 
        self.simulation_running = True




if __name__ == '__main__':
    try:
        planner = BehavioralPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
