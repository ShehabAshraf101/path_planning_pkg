#include "Grid3D.h"

using namespace planning;

template <typename T>
Grid3D<T>::Grid3D(T resolution, int grid_size, bool allow_diag_moves, T step_size, T max_lat_acc, T max_long_dec,
        T wheelbase, T rear_to_cg, int num_angle_bins, int num_actions, const std::vector<T>& steering, 
        const std::vector<T>& curvature_weights) : 
        Grid2D<T>(resolution, grid_size, allow_diag_moves), _goal_location3D(Vector3D<T>()),
        _model(step_size, max_lat_acc, max_long_dec, wheelbase, rear_to_cg, num_angle_bins, 
        num_actions, steering, curvature_weights) {}


// Public member functions
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
            && (!(this->_obstacle_map[i][j])))
        {
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
        int i1 = static_cast<int>(position._x/this->_resolution);
        int i2 = static_cast<int>(std::ceil(position._x/this->_resolution));
        int j1 = static_cast<int>(position._y/this->_resolution);
        int j2 = static_cast<int>(std::ceil(position._y/this->_resolution));
        if (this->_obstacle_map[i1][j1] || this->_obstacle_map[i1][j2] || this->_obstacle_map[i2][j1] || this->_obstacle_map[i2][j2])
        {
            return false;
        }
    }
    
    return true;
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


// Explicit instantiation of supported types
template class Grid3D<float>;
template class Grid3D<double>;