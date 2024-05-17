#include "Grid3D.h"

using namespace planning;

template <typename T>
Grid3D<T>::Grid3D(T resolution, T obstacle_threshold, T obstacle_prob_min, T obstacle_prob_max, 
        T obstacle_prob_free, int grid_size, bool allow_diag_moves, T step_size, 
        T max_lat_acc, T max_long_dec, T wheelbase, T rear_to_cg, T apf_rep_constant, 
        T apf_active_angle, int num_angle_bins, int num_actions, const std::vector<T>& steering, 
        const std::vector<T>& curvature_weights) : 
        Grid2D<T>(resolution, obstacle_threshold, obstacle_prob_min, obstacle_prob_max, 
                obstacle_prob_free, grid_size, allow_diag_moves),
        _apf_rep_constant(apf_rep_constant),
        _apf_active_angle(apf_active_angle), 
        _goal_location3D(Vector3D<T>()),
        _model(step_size, max_lat_acc, max_long_dec, wheelbase, rear_to_cg, num_angle_bins, 
                num_actions, steering, curvature_weights) {}


// Public member functions
template <typename T>
void Grid3D<T>::update_obstacles(const std::vector<Obstacle<T>>& obstacles, const std::vector<T>& confidence, 
    const T apf_added_radius)
{
    // copy obstacles positions and radii
    _apf_obstacles.clear();
    _apf_obstacles.reserve(obstacles.size());
    for (const auto& obstacle : obstacles)
    {
        // get position in the grid frame
        Vector2D<T> position(obstacle._pose2D._x - _goal_location3D._x, 
                obstacle._pose2D._y - _goal_location3D._y);
        position = position.get_rotated_vector(this->_grid_heading);
        position._x += this->_grid_size_4_5 * this->_resolution;
        position._y += this->_grid_size_2 * this->_resolution;

        _apf_obstacles.emplace_back(std::piecewise_construct, 
                std::forward_as_tuple(position._x, position._y),
                std::forward_as_tuple(std::max(obstacle._dimensions._x, obstacle._dimensions._y)/2 + apf_added_radius));
    }

    // update obstacle map using Grid2D
    Grid2D<T>::update_obstacles(obstacles, confidence);
}

template <typename T>
bool Grid3D<T>::get_neighbors(const Node3D<T>& node, std::vector<Node3D<T>>& neighbors) const
{
    // get all kinematically and dynamically feasible neighbors using vehicle model
    bool velocity_safe = _model.get_neighbors(node, neighbors);

    // ensure that all neighbors are within the boundaries of the grid and assign base nodes
    auto it = neighbors.begin();
    while (it != neighbors.end())
    {
        int i = static_cast<int>(it->_pose2D._x/this->_resolution);
        int j = static_cast<int>(it->_pose2D._y/this->_resolution);
        if ((i > -1) && (i < this->_grid_size) && (j > -1) && (j <  this->_grid_size)
            && (this->_obstacle_map[i][j] < this->_obstacle_log_threshold))
        {
            T field_cost = get_field_intensity(*it);
            it->_cost_g += field_cost;
            it->_cost_f += field_cost;
            it->_base_node = &(this->_node_map[i][j]);
            it++;
        }
        else
        {
            it = neighbors.erase(it);
        }
    }

    return velocity_safe;
}

// Returns whether the path is collision free or not (true = collision free)
template <typename T>
bool Grid3D<T>::check_path(const std::vector<Vector3D<T>>& path) const
{
    for (const auto& position : path)
    {
        // check for obstacles occupying this position
        int i1 = static_cast<int>(std::round(position._x/this->_resolution));
        int j1 = static_cast<int>(std::round(position._y/this->_resolution));
        if ((i1 < 0) || (i1 >= this->_grid_size) || (j1 < 0) || (j1 >= this->_grid_size)
            || (this->_obstacle_map[i1][j1] >= this->_obstacle_log_threshold)) 
        {
            return false;
        }
    }
    
    return true;
}

template <typename T>
Vector3D<T> Grid3D<T>::get_goal_location() const
{
    return _goal_location3D;
}

template <typename T>
Node3D<T> Grid3D<T>::update_goal_heading(const Vector3D<T>& goal, const Vector3D<T>& start)
{
    // update goal and grid heading for 2D grid
    Vector2D<T> goal_2D(goal._x, goal._y), start_2D(start._x, start._y);
    Grid2D<T>::update_goal_heading(goal_2D, start_2D);
    
    // update 3D grid parameters and return copy of goal node
    _goal_location3D = goal;
    Vector3D<T> goal_pose(this->_grid_size_4_5 * this->_resolution, 
            this->_grid_size_2 * this->_resolution, 
            wrap_pi(goal._heading - this->_grid_heading));
    
    return Node3D<T>(goal_pose, static_cast<T>(0), static_cast<T>(0), 0, 
            get_heading_index(goal_pose._heading, _model.get_precision()), 
            &(this->_node_map[this->_grid_size_4_5][this->_grid_size_2]), nullptr); 
}

template <typename T>
Node3D<T> Grid3D<T>::set_start_node(const Vector3D<T>& start)
{
    // get relative position in the grid's frame
    Vector3D<T> rel_position(start._x - _goal_location3D._x, start._y - _goal_location3D._y, 
            start._heading);
    rel_position = rel_position.get_rotated_vector(this->_grid_heading);

    // get continuous coordinates in the grid's frame
    Vector3D<T> pose2D(rel_position._x + this->_grid_size_4_5 * this->_resolution, 
            rel_position._y + this->_grid_size_2 * this->_resolution, rel_position._heading);

    // get discrete coordinates
    int i = static_cast<int>(pose2D._x/this->_resolution);
    int j = static_cast<int>(pose2D._y/this->_resolution);

    // check validity of start location
    if((i > -1) && (i < this->_grid_size) && (j > -1) && (j < this->_grid_size))
    {
        // reset the node's cost and predeccesor and return a copy of it
        (this->_node_map[i][j]).soft_reset();
        return Node3D<T>(pose2D, static_cast<T>(0), static_cast<T>(0), 
                _model.get_default_action_index(), 
                get_heading_index(pose2D._heading, _model.get_precision()),
                &(this->_node_map[i][j]), nullptr);
    }

    // Default to (0, 0, 0) node
    (this->_node_map[0][0]).soft_reset();
    Vector3D<T> pose2D_default;
    return Node3D<T>(pose2D_default, static_cast<T>(0), static_cast<T>(0), 
        _model.get_default_action_index(), 
        get_heading_index(pose2D_default._heading, _model.get_precision()),
        &(this->_node_map[0][0]), nullptr);
}

template <typename T>
const std::vector<T>& Grid3D<T>::get_abs_curvatures() const
{
    return _model.get_abs_curvatures();
}


template <typename T>
T Grid3D<T>::get_field_intensity(const Node3D<T>& node) const
{
    // define a lambda function for calculating the field of a single obstacle
    auto field_obstacle = [this, node] (T acc_sum, const std::pair<Vector2D<T>, T>& obstacle)
    {
        // calculate distance to obstacle
        T distance = std::hypot(obstacle.first._x - node._pose2D._x, obstacle.first._y - node._pose2D._y);
        T angle = std::abs(wrap_pi(node._pose2D._heading - std::atan2(obstacle.first._y - node._pose2D._y, obstacle.first._x - node._pose2D._x))); 
        angle = std::max(_apf_active_angle - angle, static_cast<T>(0.0));
        T field_potential = 0;
        if (distance < obstacle.second)
        {
            field_potential = _apf_rep_constant * std::pow((1.0/distance - 1.0/obstacle.second), 2);
            field_potential = field_potential * angle/_apf_active_angle;  
        }

        return acc_sum + field_potential;
    };

    // loop over all obstacles accumulating obstacle potential fields
    return std::accumulate(_apf_obstacles.begin(), _apf_obstacles.end(), static_cast<T>(0.0), field_obstacle);
}

// Explicit instantiation of supported types
template class Grid3D<float>;
template class Grid3D<double>;