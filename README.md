# Path Planning Package

## Package Summary

The **path_planning_pkg** contains all nodes responsible for generating the trajectory that the vehicle should ideally follow, whether offline or online. Accordingly, the package contains four nodes as the following:
* `offline_planner`: An offline planner, which was used for generating predefined trajectories of different shapes (straight, lane change, circular, infinity) in an offline manner.

* `global_planner`: An offline GPS-like global planner, responsible for finding the shortest piecewise linear path that allows the vehicle to reach its desired destination. For this, the A* algorithm is applied to a predefined graph of nodes, where each node corresponds to a connection between two roads or two section of the same road. Note that the output of this planner is neither kinematically feasible nor can it adapt to obstacles, hence it cannot be used directly for the vehicle.

* `behavioral_planner`: An intermediate planner that processes the global plan in order to extract the next waypoint for the vehicle to target as well as determining the heading that the vehicle should arrive at that waypoint with. After doing this, it sends the target pose to the local planner along with whether it should stop at that waypoint or not.

* `local_planner`: The lowest level planner, responsible for generating the actual trajectory that the vehicle will follow. It maintains a dynamic probabilistic map of the environment and updates it regularly. Then, the Hybrid A* algorithm is applied to attempt to find an obstacle-free path that connect the current and target poses. Additionally, it contains a pedestrian handler which slows down the vehicle to allow pedestrians to pass. Lastly, a velocity profile is generated for the determined path along with the velocity limit.

## Nodes Summary

### Offline Planner

#### Node Parameters:
* `velocity_max`: Maximum velocity that the generated velocity profile can reach if the path's curvature allows for it.
* `acc_long_max`: Maximum allowable longitudinal acceleration for the generated velocity profile. 
* `acc_lat_max`: Maximum allowable lateral acceleration for the generated velocity profile.
* `dec_long_max`: Maximum allowable longitudinal deceleration for the generated velocity profile.
* `path_type`: Path label which determines the type of path generated. Can be any of: *'straight'*, *'circular'*, *'lane_change'* or *'infinity'*.
* `publish_heading`: Should set to true when using along with the closed-loop controller.
* `path_length`: Length of the circular path in radians (i.e. 2pi = 1 full rotation)
* `path_length_straight`: Length of the straight path as well as being the length of the two straight section before and after the lane change maneuver. 
* `path_length_curve_x`: Length of the section over which the lane change maneuver takes place.  
* `path_radius`: Radius of the circular path and circular region of the infinity path.
* `step_size`: Step size between each two subsequent points in radians for circular or infinity paths and in meters for straight and lane change paths. 
* `rotate_ccw`: Controls the direction of rotation for the circular path.
* `lane_width`: Lateral offset for lane change maneuver (which is same as lane width).
* `turn_left`: Controls the direction of turning for the lane change maneuver. 
* `position_current`: Starting postion of the vehicle to add as an offset to the generated trajectory.

### Global Planner

#### Node Parameters:
* `pose_topic_name`: Name of the topic where the vehicle's current pose and twist are published.
* `map_type`: Determines the map to which the vehicle was deployed, in order to know which graph of nodes to use. Can be any of: *'straight'*, *'circular'*, *'lane_change'* or *'custom'*.

#### Known Issues and Possible Solutions:
* **Issue**: Not an issue but to publish waypoints when using the custom map, you should use the following command: `rostopic pub --once /global_planner/set_goal std_msgs/Char $(printf '%d' \'X)` where *X* can be replaced by the target waypoint's corresponding character.

### Behavioral Planner

#### Node Parameters:
* `pose_topic_name`: Name of the topic where the vehicle's current pose and twist are published.
* `object_topic_name`: Name of the topic to which the list of objects' bounding boxes is published.
* `global_plan_topic_name`: Name of the topic to which the global plan is published.
* `update_rate`: Update rate of the planner in Hz.
* `wp_proximity_up_threshold`: Upper threshold for the distance to the next waypoint in meters. Should **never** exceed 0.8 * the local planner's map size.
* `wp_proximity_lw_threshold`: Lower threshold for the distance to the next waypoint in meters. Should be set to give the planner a sufficient distance to plan its trajectory ahead.
* `velocity_threshold`: Absolute threshold for the vehicle velocity below which it can be considered stationary (not implemented).
* `stop_sign_time_threshold`: Time in seconds that the vehicle should remain stopped for when it encounters a stop sign (not implemented).   
* `yield_sign_time_threshold`: Time in seconds tha the vehicle should remain stopped for when it encounters a yield sign (not implemented).

#### Known Issues and Possible Solutions:
* **Issue**: Unfortunately, the behavioral planner was never completed due to time constraints. The state machine was never implemented, which limited the use of the online planner.
* **Solution**: Implement a state machine with distinct states such as: *'Drive'*, *'Decelerate to stop'* and *'Stopped'*, which would modify the global plan as needed in order to handle things like stop, yield or crosswalk signs.

### Local Planner

#### Node Parameters:
* `pose_topic_name`: Name of the topic where the vehicle's current pose and twist are published.
* `waypoint_topic_name`: Name of the topic where the next waypoint is published.
* `object_topic_name`: Name of the topic to which the list of objects' bounding boxes is published.
* `lane_topic_name`: Name of the topic where the detected lane lines are published.
#
* `grid_size`: Number of cells along each of the spatial dimensions of the grid.
* `grid_allow_diag_moves`: Determines whether to explore diagonal movements or not when applying the A* algorithm.
* `grid_resolution`: Size of each of the grid's cells (meters) along each of its spatial dimensions. 
* `obstacle_threshold`: Probability lower threshold for considering a cell in the grid as occupied by an obstacle.
* `obstacle_prob_min`: Lower saturation limit for the probability of a cell in the grid.
* `obstacle_prob_max`: Upper saturation limit for the probability of a cell in the grid.
* `obstacle_prob_free`: Probability of being occupied by which all cells that were not updated as obstacles will be updated with, in order to erase false object detections (must be lower than 0.5).
#
* `update_rate_hz`: Update rate in Hz of the planner, set to 0.1 times the rate of the controller.
* `vehicle_length`: Length of the vehicle in meters to consider when checking for collisions with other objects. 
* `vehicle_width`: Width of the vehicle in meters to add to lane lines. 
* `wheelbase`: The length of the vehicle's wheelbase in meters.
* `rear_to_cg`: The distace from the center of the rear-axle to the vehicle center of gravity along its longitudinal axis.
* `max_velocity`: Maximum velocity limit that the velocity profile is allowed to reach if Hybrid A* was successful. 
* `coast_velocity`: Maximum velocity limit that the velocity profile is allowed to reach if Hybrid A* was not successful.
* `max_lat_acc`: Maximum allowable lateral acceleration for the generated velocity profile.
* `max_long_acc`: Maximum allowable longitudinal acceleration for the generated velocity profile. 
* `max_long_dec`: Maximum allowable longitudinal deceleration for the generated velocity profile.
#
* `num_angle_bins`: Number of equally spaced sections that the 360 degrees for the vehicle's heading are divided into (i.e. grid's angular size).
* `num_actions`: Determines the number of actions (steering angles) to the left and right of the previous action to explore for generating neighboring nodes (i.e. one means that 3 actions are explored in total: the same as the previous one, the one to the left of it and the one to its right). This was implemented to prevent sudden steering changes. 
* `step_size`: The Euclidean distance between each two neighboring nodes. Must always be larger than the grid's resolution so that new nodes can be generated.
* `apf_rep_constant`: Repulsive constant of the Artificial Potential Field (APF) that is applied around obstacles to deter the vehicle away from them, while not overly enlarging them.
* `apf_active_angle`: Determines the surrounding region around the vehicle at which obstacles' field can affect it (180 degs = all around the vehicle, 90 degs = in front of the vehicle only, etc) 
* `confidence_object`: Unused parameter as objects' confidence is sent along with their bounding boxes. 
* `confidence_lane`: Probability that the detected lane lines are obstacles. Must be greater than 0.5, greater values mean that we're more confident in the lane detections. 
* `apf_object_added_radius`: Radius in (meters) that is added to the active region of any obstacles' repulsive field.
* `steering`: Set of steering angles in (degs) that are used for generating neighbors for nodes. Must be symmetrical.
* `curvature_weights`: Set of penalizing factors applied to each steering action based on its corresponding curvature. Help encourage smoother paths.
#
* `dubins_shot_interval`: Initial interval in number of iterations after which Hybrid A*'s analytic expansions (Dubins shots) are attempted.
* `dubins_shot_interval_decay`: Decay in the interval in number of iterations for applying Dubins shots after everytime they're attempted.
# 
* `detection_arc_angle_deg`: Total arc angle in (degs) that pedestrians detected inside are considered by the local planner. Cannot exceed two times the camera's horizontal perspective angle.
* `min_stop_dist`: Minimum distance threshold to a pedestrian below which the vehicle's max velocity is set to zero, in order to come to stop. 
* `min_allowable_ttc`: Minimum allowable time to collision to a pedestrian below which the vehicle should start to slow down, such that the TTC is maitained as the vehicle approaches the pedestrian.
* `max_long_dec_ph`: Maximum longitudinal decleration used when calculating the TTC to pedestrians. 
* `min_vel_ph`: Minimum velocity limit (m/s) that can applied to the vehicle based on the TTC to pedestrians, other than coming to a complete stop. 

#### Known Issues and Possible Solutions:
* **Issue**: Vehicle steers to avoid an imaginary object which is not there.
* **Solution**: This is caused by false lane detections which force the planner to steer to avoid them. They usually occur on lateral section of the dashed lanes, which after adding half the vehicle width to, force the vehicle to steering aggressively away. The only solution is to remedy the faults of the lane detector first, ideally replacing it as advised there.
* **Issue**: Vehicle struggles, fails almost always, to navigate the roundabout in the custom city map.
* **Solution**: Add four more nodes to the global planner's node graph in the roundabout to have smoother transitions between the waypoints.
* **Issue**: If a dynamic obstacle, like another car, totally obstructs the target waypoint, then Hybrid A* will keep failing until that object moves.
* **Solution**: Allow for targeting other nodes other than the goal node upon failure. This can be done by looping over a number of the explored nodes and choosing one with the lowest heuristic cost as that should be the closest to the goal node (feel free to contact me about this, I wanted to do it but wouldn't have been able to test it).
* **Issue**: Pedestrian handler stops very late when approaching pedestrians, if at all.
* **Solution**: Caused by the inconsistency of detections. Can be solved by some conditions but ideally solved by implementing a proper behavioral planner, which would add new waypoints for the vehicle to stop at before crosswalk signs.   