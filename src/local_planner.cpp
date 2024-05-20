#include "local_planner.h"

/* Base class (LocalPlannerBase) definition */

// Contructors
template <typename T>
LocalPlannerBase<T>::LocalPlannerBase(ros::NodeHandle& nh)
{
    // Declare variables needed for retrieving parameters from the ROS parameter server
    // topic names
    std::string odom_topic_name, waypoint_topic_name, object_topic_name;

    // 2D grid parameters
    int grid_size;
    bool grid_allow_diag_moves;
    T grid_resolution; 
    T obstacle_threshold, obstacle_prob_min, obstacle_prob_max, obstacle_prob_free;

    // vehicle parameters
    T vehicle_length, vehicle_width, wheelbase, rear_to_cg;
    T max_velocity, coast_velocity;
    T max_lat_acc, max_long_acc, max_long_dec;

    // 3D grid parameters
    int num_angle_bins, num_actions;
    T step_size;
    T apf_rep_constant, apf_active_angle;
    std::vector<T> steering, curvature_weights;

    // Dubins paths parameters
    int dubins_shot_interval, dubins_shot_interval_decay;

    // Load in parameters from the ROS parameter server
    // topic names
    nh.param("/local_planner/odom_topic_name", odom_topic_name, std::string("/odometry"));
    nh.param("/local_planner/waypoint_topic_name", waypoint_topic_name, std::string("/waypoints"));
    nh.param("/local_planner/object_topic_name", object_topic_name, std::string("/objects"));

    // 2D grid parameters
    nh.param("/local_planner/grid_size", grid_size, 50);
    nh.param("/local_planner/grid_allow_diag_moves", grid_allow_diag_moves, true);
    nh.param("/local_planner/grid_resolution", grid_resolution, static_cast<T>(0.25));
    nh.param("/local_planner/obstacle_threshold", obstacle_threshold, static_cast<T>(0.8));
    nh.param("/local_planner/obstacle_prob_min", obstacle_prob_min, static_cast<T>(0.05));
    nh.param("/local_planner/obstacle_prob_max", obstacle_prob_max, static_cast<T>(0.95));
    nh.param("/local_planner/obstacle_prob_free", obstacle_prob_free, static_cast<T>(0.4));

    // vehicle parameters
    nh.param("/local_planner/vehicle_length", vehicle_length, static_cast<T>(2.5));
    nh.param("/local_planner/vehicle_width", vehicle_width, static_cast<T>(1.2));
    nh.param("/local_planner/wheelbase", wheelbase, static_cast<T>(2.2));
    nh.param("/local_planner/rear_to_cg", rear_to_cg, static_cast<T>(1.0));
    nh.param("/local_planner/max_velocity", max_velocity, static_cast<T>(6.0));
    nh.param("/local_planner/coast_velocity", coast_velocity, static_cast<T>(1.5));
    nh.param("/local_planner/max_lat_acc", max_lat_acc, static_cast<T>(3.0));
    nh.param("/local_planner/max_long_acc", max_long_acc, static_cast<T>(1.5));
    nh.param("/local_planner/max_long_dec", max_long_dec, static_cast<T>(2.5));

    // 3D grid parameters
    nh.param("/local_planner/num_angle_bins", num_angle_bins, 36);
    nh.param("/local_planner/num_actions", num_actions, 3);
    nh.param("/local_planner/step_size", step_size, static_cast<T>(0.5));
    nh.param("/local_planner/apf_rep_constant", apf_rep_constant, static_cast<T>(1.0));
    nh.param("/local_planner/apf_active_angle", apf_active_angle, static_cast<T>(45.0));
    nh.param("/local_planner/confidence_object", _confidence_object, static_cast<T>(0.7));
    nh.param("/local_planner/confidence_lane", _confidence_lane, static_cast<T>(0.55));
    nh.param("/local_planner/apf_object_added_radius", _apf_object_added_radius, static_cast<T>(2.5));
    nh.getParam("/local_planner/steering", steering);
    nh.getParam("/local_planner/curvature_weights", curvature_weights);

    // Dubins paths parameters
    nh.param("/local_planner/dubins_shot_interval", dubins_shot_interval, 250);
    nh.param("/local_planner/dubins_shot_interval_decay", dubins_shot_interval_decay, 20);

    // Report the values of all parameters
    ROS_INFO("Local Planner Parameters:");

    ROS_INFO("odom_topic_name = %s", odom_topic_name.c_str());
    ROS_INFO("waypoint_topic_name = %s", waypoint_topic_name.c_str());
    ROS_INFO("object_topic_name = %s", object_topic_name.c_str());

    ROS_INFO("grid_size = %d", grid_size);
    ROS_INFO("grid_allow_diag_moves = %s", (grid_allow_diag_moves) ? "True" : "False");
    ROS_INFO("grid_resolution = %.4f", grid_resolution);
    ROS_INFO("obstacle_threshold = %.4f", obstacle_threshold);
    ROS_INFO("obstacle_prob_min = %.4f", obstacle_prob_min);
    ROS_INFO("obstacle_prob_max = %.4f", obstacle_prob_max);
    ROS_INFO("obstacle_prob_free = %.4f", obstacle_prob_free);

    ROS_INFO("vehicle_length = %.4f", vehicle_length);
    ROS_INFO("vehicle_width = %.4f", vehicle_width);
    ROS_INFO("wheelbase = %.4f", wheelbase);
    ROS_INFO("rear_to_cg = %.4f", rear_to_cg);
    ROS_INFO("max_velocity = %.4f", max_velocity);
    ROS_INFO("coast_velocity = %.4f", coast_velocity);
    ROS_INFO("max_lat_acc = %.4f", max_lat_acc);
    ROS_INFO("max_long_acc = %.4f", max_long_acc);
    ROS_INFO("max_long_dec = %.4f", max_long_dec);

    ROS_INFO("num_angle_bins = %d", num_angle_bins);
    ROS_INFO("num_actions = %d", num_actions);
    ROS_INFO("step_size = %.4f", step_size);
    ROS_INFO("apf_rep_constant = %.4f", apf_rep_constant);
    ROS_INFO("apf_active_angle = %.4f", apf_active_angle);
    ROS_INFO("confidence_object = %.4f", _confidence_object);
    ROS_INFO("confidence_lane = %.4f", _confidence_lane);
    ROS_INFO("apf_object_added_radius = %.4f", _apf_object_added_radius);

    std::stringstream ss;
    ss << "steering (deg) = ";
    std::copy(steering.begin(), steering.end(), std::ostream_iterator<T>(ss, " "));
    ss << std::endl;
    ROS_INFO_STREAM(ss.str());
    ss.str("curvature_weights = ");
    std::copy(curvature_weights.begin(), curvature_weights.end(), std::ostream_iterator<T>(ss, " "));
    ss << std::endl;
    ROS_INFO_STREAM(ss.str());
    
    ROS_INFO("dubins_shot_interval = %d", dubins_shot_interval);
    ROS_INFO("dubins_shot_interval_decay = %d", dubins_shot_interval_decay);

    // Convert all angles from degrees to radians
    T deg_to_rad = M_PI/180.0;
    apf_active_angle *= deg_to_rad;
    std::transform(steering.begin(), steering.end(), steering.begin(), 
            [deg_to_rad] (T angle) -> T { return angle * deg_to_rad; });
    
    // Initialize class members (Non-ROS)
    _vehicle_length_2 = vehicle_length/2;
    _vehicle_width_2 = vehicle_width/2;
    _vehicle_cg_to_front = rear_to_cg;
    _velocity = 0;
    _waypoint_received = false;
    _pose = {0, 0, 0};
    _waypoint_pair.first = {0, 0, 0};
    _waypoint_pair.second = false;
    _hybrid_astar = std::make_unique<HybridAStar<T>>(dubins_shot_interval, dubins_shot_interval_decay, 
            grid_resolution, obstacle_threshold, obstacle_prob_min, obstacle_prob_max, obstacle_prob_free,
            grid_size, grid_allow_diag_moves, step_size, max_lat_acc, max_long_dec, wheelbase, rear_to_cg, 
            apf_rep_constant, apf_active_angle, num_angle_bins, num_actions, steering, curvature_weights);
    _velocity_generator = std::make_unique<VelocityGenerator<T>>(max_velocity, coast_velocity, 
            max_lat_acc, max_long_acc, max_long_dec);
    _hybrid_astar->update_goal(Vector3D<T>(0, 0, 0), Vector3D<T>(0, 0, 0));

    // Initialize ROS publishers and subscribers
    _odom_sub       = nh.subscribe(odom_topic_name, 0, &LocalPlannerBase::callback_odom, this);
    _waypoint_sub   = nh.subscribe(waypoint_topic_name, 0, &LocalPlannerBase::callback_waypoint, this);
    _object_sub     = nh.subscribe(object_topic_name, 0, &LocalPlannerBase::callback_objects, this);
}


// Public member fuctions
template <typename T>
void LocalPlannerBase<T>::callback_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
    // store pose2D and velocity
    Vector3D<T> pose = pose_to_vector3d<T>(msg->pose.pose);
    _pose = {pose._y, 
            -pose._x, 
            pose._heading};
    // std::cout << _pose._x << ", " << _pose._y << ", " << _pose._heading << "\n";   
    _velocity = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    // initialize previous path if no waypoints have been initialized yet
    if (!_waypoint_received)
    {
        _path_prev.resize(1);
        _curvature_prev.resize(1);
        _path_prev[0] = _pose;
        _curvature_prev[0] = static_cast<T>(0.0);
    }
}

template <typename T>
void LocalPlannerBase<T>::callback_waypoint(const path_planning_pkg::Waypoint::ConstPtr &msg)
{
    // store waypoint and boolean flag pair (true = stop at waypoint)
    _waypoint_pair.first = pose_to_vector3d<T>(msg->pose);
    _waypoint_pair.second = msg->stop_at_waypoint;
    // std::cout << _waypoint_pair.first._x << ", " << _waypoint_pair.first._y 
    //         << ", " << _waypoint_pair.first._heading << "\n";   

    // update goal for hybrid A* and reset the algorithm's map
    _hybrid_astar->reset();
    _hybrid_astar->update_goal(_waypoint_pair.first, _pose);

    // signify that at least a single waypoint has been received
    _waypoint_received = true;
}

template <typename T>
void LocalPlannerBase<T>::callback_objects(const perception_pkg::bounding_box_array::ConstPtr &msg)
{
    // initialize the expected object class names
    const std::vector<std::string> class_names({"car", "plastic lane barrier", "person"});

    // initialize vectors to store the obstacles and their confidence values
    std::vector<Obstacle<T>> obstacles;
    std::vector<T> confidence;
    obstacles.reserve(msg->bbs_array.size());
    confidence.reserve(msg->bbs_array.size());

    // loop over obstacles to copy their contents
    for (const auto& obstacle : msg->bbs_array)
    {
        
        // handle barriers and vehicles by enlarging size using the vehicle's length
        if ((obstacle.class_name == class_names[0]) || (obstacle.class_name == class_names[1]))
        {
            T obstacle_dim = std::max(obstacle.length, obstacle.width) + _vehicle_width_2;
            obstacles.emplace_back(obstacle.centroid.x, obstacle.centroid.y, obstacle_dim, obstacle_dim);
            confidence.emplace_back(static_cast<T>(0.5) + obstacle.confidence/2);
        }
        // else if (obstacle.class_name != class_names[2])
        // {
        //     T obstacle_dim = std::min(obstacle.length, obstacle.width);
        //     obstacles.emplace_back(obstacle.centroid.x, obstacle.centroid.y, obstacle_dim, obstacle_dim);
        //     confidence.emplace_back(static_cast<T>(0.5) + obstacle.confidence/2);
        // }
    }

    // update map using obstacles
    _hybrid_astar->update_obstacles(obstacles, confidence, _apf_object_added_radius);
}

// Protected member functions
template <typename T>
void LocalPlannerBase<T>::copy_path(const std::vector<Vector3D<T>>& path, const std::vector<T>& curvature)
{
    _path_prev.resize(path.size());
    _curvature_prev.resize(curvature.size());
    for (std::size_t i = 0; i < path.size(); i++)
    {
        _path_prev[i] = path[i];
        _curvature_prev[i] = curvature[i];
    }
}

/* Define class specialization of local planner for float datatype */

// Constructors
LocalPlanner<float>::LocalPlanner(ros::NodeHandle &nh) : 
        LocalPlannerBase(nh)
{
    // Load in remaining parameters
    std::string lane_topic_name;
    nh.param("/local_planner/lane_topic_name", lane_topic_name, std::string("/lanes"));
    ROS_INFO("lane_topic_name = %s", lane_topic_name.c_str());

    // Initialize trajectory publisher and lane subscriber
    _trajectory_pub = nh.advertise<std_msgs::Float32MultiArray>("/local_planner/trajectory", 0, true);
    _lane_sub = nh.subscribe(lane_topic_name, 0, &LocalPlanner::callback_lane, this); 
}

// Public member functions (type-specific implementations)
void LocalPlanner<float>::callback_lane(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    // ensure validity of message layout
    if((msg->layout.dim.size() < 2) || (msg->layout.dim[1].size < 4))
    {
        ROS_INFO("Lane message is incorrectly formatted");
        return;
    }

    // update trajectory only if a waypoint is available
    if (_waypoint_received)
    {
        // copy lines from serialized vector into point pairs (x1, y1, x2, y2) for each line
        std::size_t num_lines = msg->layout.dim[0].size;
        std::vector<std::pair<Vector2D<float>, Vector2D<float>>> lines;
        lines.reserve(num_lines);
        for (std::size_t i = 0; i < num_lines; i++)
        {
            lines.emplace_back(std::piecewise_construct, 
                    std::forward_as_tuple(msg->data[4 * i], msg->data[4 * i + 1]),
                    std::forward_as_tuple(msg->data[4 * i + 2], msg->data[4 * i + 3]));
        }

        // update map and path
        _hybrid_astar->update_obstacles(lines, std::vector<float>(num_lines, _confidence_lane), _vehicle_width_2);
        _hybrid_astar->update_obstacles();
        std::vector<float> curvature, velocity;
        std::vector<Vector3D<float>> path;
        std::pair<float, bool> result = _hybrid_astar->find_path(_velocity, _pose, path, curvature);

        // update velocity profile and publish new trajectory
        bool success;
        if (result.second)
        {
            // update velocity profile using new path
            success = _velocity_generator->generate_velocity_profile(_velocity, path, curvature, 
                    velocity, !result.second, _waypoint_pair.second);
            publish_trajectory(path, velocity);
            copy_path(path, curvature);
        }
        else
        {
            // update velocity profile using old path and publish trajectory
            success = _velocity_generator->generate_velocity_profile(_velocity, _path_prev, _curvature_prev, 
                    velocity, !result.second, _waypoint_pair.second);
            publish_trajectory(_path_prev, velocity);
            ROS_INFO("Hybrid A*: Failed");
        }

        if (!success)
        {
            ROS_INFO("Velocity Generator: Failed");
        }
    }
}

void LocalPlanner<float>::publish_trajectory(const std::vector<Vector3D<float>> &path, 
        const std::vector<float> &velocity) const
{
    // initialize multiarray message
    std_msgs::Float32MultiArray msg;
    msg.layout.data_offset = 0;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    
    msg.layout.dim[0].label = std::string("samples");
    msg.layout.dim[0].size = path.size();
    msg.layout.dim[0].stride = 4 * path.size();

    msg.layout.dim[1].label = std::string("x,y,heading,velocity");
    msg.layout.dim[1].size = 4;
    msg.layout.dim[1].stride = 4;

    // insert path and velocity into data vector
    msg.data.reserve(4 * path.size());
    for (auto it = path.rbegin(); it != path.rend(); it++) { msg.data.push_back(it->_x); }
    for (auto it = path.rbegin(); it != path.rend(); it++) { msg.data.push_back(it->_y); }
    for (auto it = path.rbegin(); it != path.rend(); it++) { msg.data.push_back(it->_heading); }
    msg.data.insert(msg.data.begin() + 3 * path.size(), velocity.begin(), velocity.end());

    // publish message
    _trajectory_pub.publish(msg);
}


/* Define class specialization of local planner for double datatype */

// Constructors
LocalPlanner<double>::LocalPlanner(ros::NodeHandle &nh) :
        LocalPlannerBase(nh)
{
    // Load in remaining parameters
    std::string lane_topic_name;
    nh.param("/local_planner/lane_topic_name", lane_topic_name, std::string("/lanes"));
    ROS_INFO("lane_topic_name = %s", lane_topic_name.c_str());

    // Initialize trajectory publisher and lane subscriber
    _trajectory_pub = nh.advertise<std_msgs::Float64MultiArray>("/local_planner/trajectory", 0, true);
    _lane_sub = nh.subscribe(lane_topic_name, 0, &LocalPlanner::callback_lane, this); 
}

// Public member functions (type-specific implementations)
void LocalPlanner<double>::callback_lane(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // ensure validity of message layout
    if((msg->layout.dim.size() < 2) || (msg->layout.dim[1].size < 4))
    {
        ROS_INFO("Lane message is incorrectly formatted");
        return;
    }

    // update trajectory only if a waypoint is available
    if (_waypoint_received)
    {
        // copy lines from serialized vector into point pairs (x1, y1, x2, y2) for each line
        std::size_t num_lines = msg->layout.dim[0].size;
        std::vector<std::pair<Vector2D<double>, Vector2D<double>>> lines;
        lines.reserve(num_lines);
        for (std::size_t i = 0; i < num_lines; i++)
        {
            lines.emplace_back(std::piecewise_construct, 
                    std::forward_as_tuple(msg->data[4 * i], msg->data[4 * i + 1]),
                    std::forward_as_tuple(msg->data[4 * i + 2], msg->data[4 * i + 3]));
        }

        // update map and path
        _hybrid_astar->update_obstacles(lines, std::vector<double>(num_lines, _confidence_lane), _vehicle_width_2);
        _hybrid_astar->update_obstacles();
        std::vector<double> curvature;
        std::vector<Vector3D<double>> path;
        std::pair<double, bool> result = _hybrid_astar->find_path(_velocity, _pose, path, curvature);
        if (result.second)
        {
            // update velocity profile and publish trajectory
            std::vector<double> velocity;
            bool success = _velocity_generator->generate_velocity_profile(_velocity, path, curvature, 
                    velocity, !result.second, _waypoint_pair.second);
            publish_trajectory(path, velocity);  
            if (!success)
            {
                ROS_INFO("Velocity Generator: Failed");
            } 
        }
        else
        {
            ROS_INFO("Hybrid A*: Failed");
        }
    }  
}

void LocalPlanner<double>::publish_trajectory(const std::vector<Vector3D<double>> &path, 
        const std::vector<double> &velocity) const
{
    // initialize multiarray message
    std_msgs::Float64MultiArray msg;
    msg.layout.data_offset = 0;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    
    msg.layout.dim[0].label = std::string("samples");
    msg.layout.dim[0].size = path.size();
    msg.layout.dim[0].stride = 4 * path.size();

    msg.layout.dim[1].label = std::string("x,y,heading,velocity");
    msg.layout.dim[1].size = 4;
    msg.layout.dim[1].stride = 4;

    // insert path and velocity into data vector
    msg.data.reserve(4 * path.size());
    for (auto it = path.rbegin(); it != path.rend(); it++) { msg.data.push_back(it->_x); }
    for (auto it = path.rbegin(); it != path.rend(); it++) { msg.data.push_back(it->_y); }
    for (auto it = path.rbegin(); it != path.rend(); it++) { msg.data.push_back(it->_heading); }
    msg.data.insert(msg.data.begin() + 3 * path.size(), velocity.begin(), velocity.end());

    // publish message
    _trajectory_pub.publish(msg);
}

// End of class definitions

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh;

    LocalPlanner<float> local_planner(nh);
    ros::spin();

    return 0;
}