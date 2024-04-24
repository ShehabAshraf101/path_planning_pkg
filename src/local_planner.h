#ifndef LOCAL_PLANNER
#define LOCAL_PLANNER

#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <algorithm>
#include <numeric>
#include "HybridAStar.h"
#include "VelocityGenerator.h"
#include "common.h"

#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "path_planning_pkg/Waypoint.h"

using namespace planning;

/* Helper function for extracting a Vector3D representing pose2D from goemetry_msgs/Pose msg */
template <typename T>
Vector3D<T> pose_to_vector3d(const geometry_msgs::Pose &pose)
{
    // convert to tf quaternion then rotation matrix
    tf::Quaternion quat_tf(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 rot_mat_tf(quat_tf);

    // extract yaw (heading) angle from the rotation matrix
    double roll, pitch, yaw;
    rot_mat_tf.getRPY(roll, pitch, yaw);

    // return Vector3D containing x, y, and heading angle
    return {static_cast<T>(pose.position.x),
            static_cast<T>(pose.position.y),
            static_cast<T>(yaw)};
}


/* Base class for local planner node (shared between float and double types) */ 
template <typename T>
class LocalPlannerBase
{

public:
    // Contructors
    LocalPlannerBase(ros::NodeHandle& nh);

    // Public member fuctions
    void callback_odom(const nav_msgs::Odometry::ConstPtr &msg);
    void callback_waypoint(const path_planning_pkg::Waypoint::ConstPtr &msg);
    // add callback for object detector when msg is available

protected:
    // Protected class members
    int _lane_sampling_size;                                    // Number of points by which the lanes are sampled into rectangular obstacles
                                                                // i.e. each N points are clustered together as an obstacle
    T _vehicle_length_2;                                        // Vehicle's length/2 (m) + tolerance to add to dimensions of obstacles
    T _velocity;                                                // Latest estimate of the vehicle's velocity
    Vector3D<T> _pose;                                          // Latest estimate of the vehicle's pose (pose2D)
    std::pair<Vector3D<T>, bool> _waypoint_pair;                // Next waypoint (pose2D) and flag to decide if vehicle should stop at waypoint
    std::vector<Obstacle<T>> _object_obstacles;                 // Vector of all objects to be added to the grid before search begins
    std::vector<Obstacle<T>> _boundary_obstacles;               // Vector of all piecewise obstacles representing the boundaries of drivable area 
    std::unique_ptr<HybridAStar<T>> _hybrid_astar;              // Path planner for kinematically feasbile obstacle-free paths using Hybrid A*
    std::unique_ptr<VelocityGenerator<T>> _velocity_generator;  // Responsible for generating a velocity profile given a path and vehicle's limits

    ros::Publisher _trajectory_pub;                             // ROS publisher of the trajectory msg as Float32 or Float64MultiArray
    ros::Subscriber _odom_sub;                                  // ROS subscriber for the odometry msg holding the vehicle's state
    ros::Subscriber _waypoint_sub;                              // ROS subscriber for custom waypoint msg
    ros::Subscriber _object_sub;                                // ROS subscriber for objects detected by camera
    ros::Subscriber _lane_sub;                                  // ROS subscriber for Float32 or Float64MultiArray msg holding the coordinates
                                                                // of the left and right road boundaries
};


/* Derive local planner class from the above base class for all types */
template <typename T>
class LocalPlanner : public LocalPlannerBase<T> 
{

public:
    // Constructors
    LocalPlanner(ros::NodeHandle &nh) : LocalPlannerBase<T>(nh) {};
};


/* Declare class specialization of local planner for float datatype */
template <>
class LocalPlanner<float> : public LocalPlannerBase<float>
{

public:
    // Constructors
    LocalPlanner(ros::NodeHandle &nh);

    // Public member functions (type-specific implementations)
    void callback_lane(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void publish_trajectory(std::vector<Vector3D<float>> &path, std::vector<float> &velocity);
};


/* Declare class specialization of local planner for double datatype */
template <>
class LocalPlanner<double> : public LocalPlannerBase<double>
{

public:
    // Constructors
    LocalPlanner(ros::NodeHandle &nh);

    // Public member functions (type-specific implementations)
    void callback_lane(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void publish_trajectory(std::vector<Vector3D<double>> &path, std::vector<double> &velocity);
};

#endif