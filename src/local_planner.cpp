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

    // vehicle parameters
    T vehicle_length, wheelbase, rear_to_cg;
    T max_velocity, coast_velocity;
    T max_lat_acc, max_long_acc, max_long_dec;

    // 3D grid parameters
    int num_angle_bins, num_actions;
    T step_size;
    std::vector<T> steering, curvature_weights;

    // Dubins paths parameters
    int dubins_shot_interval, dubins_shot_interval_decay;

    // Load in parameters from the ROS parameter server
    // topic names
    nh.param("/local_planner/odom_topic_name", odom_topic_name, std::string("/odometry"));
    nh.param("/local_planner/waypoint_topic_name", waypoint_topic_name, std::string("/waypoints"));
    nh.param("/local_planner/object_topic_name", object_topic_name, std::string("/objects"));

    // 2D grid parameters
    nh.param("/local_planner/lane_sampling_size", _lane_sampling_size, 50);
    nh.param("/local_planner/grid_size", grid_size, 50);
    nh.param("/local_planner/grid_allow_diag_moves", grid_allow_diag_moves, true);
    nh.param("/local_planner/grid_resolution", grid_resolution, static_cast<T>(0.25));

    // vehicle parameters
    nh.param("/local_planner/vehicle_length", vehicle_length, static_cast<T>(2.5));
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

    ROS_INFO("lane_sampling_size = %d", _lane_sampling_size);
    ROS_INFO("grid_size = %d", grid_size);
    ROS_INFO("grid_allow_diag_moves = %s", (grid_allow_diag_moves) ? "True" : "False");
    ROS_INFO("grid_resolution = %.4f", grid_resolution);

    ROS_INFO("vehicle_length = %.4f", vehicle_length);
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

    // Convert steering to radians
    T deg_to_rad = M_PI/180.0;
    std::transform(steering.begin(), steering.end(), steering.begin(), 
            [deg_to_rad] (T angle) -> T { return angle * deg_to_rad; });

    // Initialize class members (Non-ROS)
    _vehicle_length_2 = vehicle_length/2;
    _velocity = 0;
    _pose = {0, 0, 0};
    _waypoint_pair.first = {0, 0, 0};
    _waypoint_pair.second = false;
    _hybrid_astar = std::make_unique<HybridAStar<T>>(dubins_shot_interval, dubins_shot_interval_decay, 
            grid_resolution, grid_size, grid_allow_diag_moves, step_size, max_lat_acc, max_long_dec, 
            wheelbase, rear_to_cg, num_angle_bins, num_actions, steering, curvature_weights);
    _velocity_generator = std::make_unique<VelocityGenerator<T>>(max_velocity, coast_velocity, 
            max_lat_acc, max_long_acc, max_long_dec);

    // Initialize ROS publishers and subscribers
    _odom_sub       = nh.subscribe(odom_topic_name, 0, &LocalPlannerBase::callback_odom, this);
    _waypoint_sub   = nh.subscribe(waypoint_topic_name, 1, &LocalPlannerBase::callback_waypoint, this);
    // add object subscriber when available
}


// Public member fuctions
template <typename T>
void LocalPlannerBase<T>::callback_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
    // store pose2D and velocity
    _pose = pose_to_vector3d<T>(msg->pose.pose);
    _velocity = msg->twist.twist.linear.x;
}

template <typename T>
void LocalPlannerBase<T>::callback_waypoint(const path_planning_pkg::Waypoint::ConstPtr &msg)
{
    // store waypoint and boolean flag pair (true = stop at waypoint)
    _waypoint_pair.first = pose_to_vector3d<T>(msg->pose);
    _waypoint_pair.second = msg->stop_at_waypoint;
}

// add callback for object detector when msg is available


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
    // ensure validity of message
    if(msg->layout.dim.size() < 2)
    {
        return;
    }

    // erase old boundaries
    _boundary_obstacles.clear();

    // group sets of points from each boundary together
    // first boundary
    int size_1 = msg->layout.dim[0].size; // number of points in first boundary 
    int num_obstacles = static_cast<int>(std::ceil(size_1/_lane_sampling_size));
    for (int i = 0; i < num_obstacles; i++)
    {
        // get min, max and mean for each set of points along both axes
        auto it_start_x = msg->data.begin() + i * _lane_sampling_size;
        auto it_start_y = msg->data.begin() + size_1 + i * _lane_sampling_size;
        float max_x, max_y, min_x, min_y, mean_x, mean_y;
        if (i != (num_obstacles - 1))
        {
            auto it_end_x = msg->data.begin() + (i+1) * _lane_sampling_size;
            auto it_end_y = msg->data.begin() + size_1 + (i+1) * _lane_sampling_size;
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/_lane_sampling_size;
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/_lane_sampling_size;
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }
        else
        {
            auto it_end_x = msg->data.begin() + size_1;
            auto it_end_y = msg->data.begin() + 2 * size_1;
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/(size_1 - i * _lane_sampling_size);
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/(size_1 - i * _lane_sampling_size);
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }

        // initialize obstacles from boundary data
        _boundary_obstacles.emplace_back(mean_x, mean_y, (max_x - min_x) + _vehicle_length_2, (max_y - min_y) + _vehicle_length_2);
    }

    // second boundary
    int size_2 = msg->layout.dim[1].size; // number of points in second boundary 
    num_obstacles = static_cast<int>(std::ceil(size_2/_lane_sampling_size));
    for (int i = 0; i < num_obstacles; i++)
    {
        // get min, max and mean for each set of points along both axes
        auto it_start_x = msg->data.begin() + 2 * size_1 + i * _lane_sampling_size;
        auto it_start_y = msg->data.begin() + 2 * size_1 + size_2 + i * _lane_sampling_size;
        float max_x, max_y, min_x, min_y, mean_x, mean_y;
        if (i != (num_obstacles - 1))
        {
            auto it_end_x = msg->data.begin() + 2 * size_1 + (i+1) * _lane_sampling_size;
            auto it_end_y = msg->data.begin() + 2 * size_1 + size_2 + (i+1) * _lane_sampling_size;
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/_lane_sampling_size;
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/_lane_sampling_size;
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }
        else
        {
            auto it_end_x = msg->data.begin() + 2 * size_1 + size_2;
            auto it_end_y = msg->data.end();
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/(size_2 - i * _lane_sampling_size);
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/(size_2 - i * _lane_sampling_size);
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }

        // initialize obstacles from boundary data
        _boundary_obstacles.emplace_back(mean_x, mean_y, (max_x - min_x) + _vehicle_length_2, (max_y - min_y) + _vehicle_length_2);
    }
}

void LocalPlanner<float>::publish_trajectory(std::vector<Vector3D<float>> &path, 
        std::vector<float> &velocity)
{
    // initialize multiarray message
    std_msgs::Float32MultiArray msg;
    msg.layout.data_offset = 0;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    
    msg.layout.dim[0].label = std::string("samples");
    msg.layout.dim[0].size = path.size();
    msg.layout.dim[0].stride = 4 * path.size();

    msg.layout.dim[1].label = std::string("x,y,heading,curvature");
    msg.layout.dim[1].size = 4;
    msg.layout.dim[1].stride = 4;

    // insert path and velocity into data vector
    msg.data.resize(4 * path.size());
    for (const auto& pose : path) { msg.data.push_back(pose._x); }
    for (const auto& pose : path) { msg.data.push_back(pose._y); }
    for (const auto& pose : path) { msg.data.push_back(pose._heading); }
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
    // ensure validity of message
    if(msg->layout.dim.size() < 2)
    {
        return;
    }

    // erase old boundaries
    _boundary_obstacles.clear();

    // group sets of points from each boundary together
    // first boundary
    int size_1 = msg->layout.dim[0].size; // number of points in first boundary 
    int num_obstacles = static_cast<int>(std::ceil(size_1/_lane_sampling_size));
    for (int i = 0; i < num_obstacles; i++)
    {
        // get min, max and mean for each set of points along both axes
        auto it_start_x = msg->data.begin() + i * _lane_sampling_size;
        auto it_start_y = msg->data.begin() + size_1 + i * _lane_sampling_size;
        double max_x, max_y, min_x, min_y, mean_x, mean_y;
        if (i != (num_obstacles - 1))
        {
            auto it_end_x = msg->data.begin() + (i+1) * _lane_sampling_size;
            auto it_end_y = msg->data.begin() + size_1 + (i+1) * _lane_sampling_size;
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/_lane_sampling_size;
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/_lane_sampling_size;
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }
        else
        {
            auto it_end_x = msg->data.begin() + size_1;
            auto it_end_y = msg->data.begin() + 2 * size_1;
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/(size_1 - i * _lane_sampling_size);
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/(size_1 - i * _lane_sampling_size);
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }

        // initialize obstacles from boundary data
        _boundary_obstacles.emplace_back(mean_x, mean_y, max_x - min_x, max_y - min_y);
    }

    // second boundary
    int size_2 = msg->layout.dim[1].size; // number of points in second boundary 
    num_obstacles = static_cast<int>(std::ceil(size_2/_lane_sampling_size));
    for (int i = 0; i < num_obstacles; i++)
    {
        // get min, max and mean for each set of points along both axes
        auto it_start_x = msg->data.begin() + 2 * size_1 + i * _lane_sampling_size;
        auto it_start_y = msg->data.begin() + 2 * size_1 + size_2 + i * _lane_sampling_size;
        double max_x, max_y, min_x, min_y, mean_x, mean_y;
        if (i != (num_obstacles - 1))
        {
            auto it_end_x = msg->data.begin() + 2 * size_1 + (i+1) * _lane_sampling_size;
            auto it_end_y = msg->data.begin() + 2 * size_1 + size_2 + (i+1) * _lane_sampling_size;
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/_lane_sampling_size;
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/_lane_sampling_size;
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }
        else
        {
            auto it_end_x = msg->data.begin() + 2 * size_1 + size_2;
            auto it_end_y = msg->data.end();
            auto min_max_pair_x = std::minmax_element(it_start_x, it_end_x);
            auto min_max_pair_y = std::minmax_element(it_start_y, it_end_y);
            mean_x = std::accumulate(it_start_x, it_end_x, 0)/(size_2 - i * _lane_sampling_size);
            mean_y = std::accumulate(it_start_y, it_end_y, 0)/(size_2 - i * _lane_sampling_size);
            max_x = *(min_max_pair_x.second); 
            min_x = *(min_max_pair_x.first);
            max_y = *(min_max_pair_y.second);
            min_y = *(min_max_pair_y.first);
        }

        // initialize obstacles from boundary data
        _boundary_obstacles.emplace_back(mean_x, mean_y, max_x - min_x, max_y - min_y);
    }
}

void LocalPlanner<double>::publish_trajectory(std::vector<Vector3D<double>> &path, 
        std::vector<double> &velocity)
{
    // initialize multiarray message
    std_msgs::Float64MultiArray msg;
    msg.layout.data_offset = 0;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    
    msg.layout.dim[0].label = std::string("samples");
    msg.layout.dim[0].size = path.size();
    msg.layout.dim[0].stride = 4 * path.size();

    msg.layout.dim[1].label = std::string("x,y,heading,curvature");
    msg.layout.dim[1].size = 4;
    msg.layout.dim[1].stride = 4;

    // insert path and velocity into data vector
    msg.data.resize(4 * path.size());
    for (const auto& pose : path) { msg.data.push_back(pose._x); }
    for (const auto& pose : path) { msg.data.push_back(pose._y); }
    for (const auto& pose : path) { msg.data.push_back(pose._heading); }
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