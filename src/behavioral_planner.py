#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from path_planning_pkg.msg import Waypoint

# Define enum for different states
class State:
    DRIVE = 0
    DECELERATE = 1
    STOP = 2

class BehavioralPlanner:
    def __init__(self):
        rospy.init_node('behavioral_planner', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_state = State.STOP  # Initialize the state to STOP
        self.current_velocity = float
        self.velocity_threshold = 0.1
        self.stop_start_time = None  # Initialize stop state start time
        self.simulation_running = False
        # Detection flags 
        self.stop_sign = False
        self.cross_walk = False
        self.pedestrian_on_road = False
        self.yield_sign = False
        self.car_detected = False
        self.final_waypoint = False

        # Subscriber for object detection
        rospy.Subscriber("object_detection", Image, self.object_detection_callback)

        # Subscriber for global plan
        rospy.Subscriber("/global_plan", Waypoint, self.global_plan_callback)

         # Odometry subscriber
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def run(self):
        while not rospy.is_shutdown():
            while not self.simulation_running:
                pass
            self.execute_state()
            self.update_state()
            self.rate.sleep()

    def update_state(self):
        if self.current_state == State.DRIVE:
            if self.stop_sign or (self.cross_walk and self.pedestrian_on_road) or (self.yield_sign and self.car_detected) or self.final_waypoint:
                self.current_state = State.DECELERATE
        elif self.current_state == State.DECELERATE:
            if self.current_velocity <= self.velocity_threshold:
                self.current_state = State.STOP
        elif self.current_state == State.STOP:
            if self.stop_start_time is None:
                self.stop_start_time = rospy.Time.now().secs  # Start counting time when entering stop state
            elapsed_time = rospy.Time.now().secs - self.stop_start_time  # Calculate elapsed time
            if (self.cross_walk and not self.pedestrian_on_road) or (self.stop_sign and elapsed_time > 5) or (not self.stop_sign and not self.cross_walk and not self.yield_sign) or (self.yield_sign and not self.car_detected):
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

    def object_detection_callback(self, data):
        # Process the object detection data received in 'data' message and update the variables accordingly
        # Example:
        # if data.stop_sign_detected:
        #     self.stop_sign = True
        # if data.cross_walk_detected:
        #     self.cross_walk = True
        # ...
        pass
    
    def global_plan_callback(self, data):
        self.final_waypoint = data.stop_at_waypoint

    # Callback function to handle incoming Odometry messages
    def odom_callback(self, msg):
        self.current_velocity = math.sqrt((msg.twist.twist.linear.x)**2 + (msg.twist.twist.linear.y)**2)
        self.simulation_running = True



if __name__ == '__main__':
    try:
        planner = BehavioralPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
