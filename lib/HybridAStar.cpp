#include "HybridAStar.h"

using namespace planning;

// Contructors
template <typename T>
HybridAStar<T>::HybridAStar(int dubins_shot_interval, int dubins_shot_interval_decay, T grid_resolution, 
        T obstacle_threshold, T obstacle_prob_min, T obstacle_prob_max, T obstacle_prob_free, 
        int grid_size, bool grid_2d_allow_diag_moves, T step_size, T max_lat_acc, 
        T max_long_dec, T wheelbase, T rear_to_cg, T apf_rep_constant, T apf_active_angle, 
        int num_angle_bins, int num_actions, const std::vector<T>& steering, 
        const std::vector<T>& curvature_weights) : 
        _dubins_shot_interval(dubins_shot_interval), 
        _dubins_shot_interval_decay(dubins_shot_interval_decay),
        _dubins_shot_successful(false),
        _goal_node(Node3D<T>()),
        _terminal_node(Node3D<T>()),
        _grid(grid_resolution, obstacle_threshold, obstacle_prob_min, obstacle_prob_max, 
                obstacle_prob_free, grid_size, grid_2d_allow_diag_moves, step_size, 
                max_lat_acc, max_long_dec, wheelbase, rear_to_cg, apf_rep_constant, 
                apf_active_angle, num_angle_bins, num_actions, steering, curvature_weights), 
        _astar(_grid), 
        _dubins(wheelbase/(std::cos(std::atan2(rear_to_cg * tan_max<T>(steering), wheelbase)) * 
            tan_max<T>(steering)), step_size) {}


// Public member functions
template <typename T>
void HybridAStar<T>::update_obstacles(const std::vector<Obstacle<T>>& obstacles, const std::vector<T>& confidence, 
        const T apf_added_radius)
{
    _grid.update_obstacles(obstacles, confidence, apf_added_radius);
}

template <typename T>
void HybridAStar<T>::update_obstacles(const std::vector<std::pair<Vector2D<T>, Vector2D<T>>>& lines, 
        const std::vector<T>& confidence, const T line_width)
{
    _grid.update_obstacles(lines, confidence, line_width);
}

template <typename T>
void HybridAStar<T>::update_obstacles()
{
    _grid.update_obstacles();
}

template <typename T>
void HybridAStar<T>::reset()
{
    _astar.reset();
}

template <typename T>
void HybridAStar<T>::update_goal(const Vector3D<T>& goal, const Vector3D<T>& start)
{
    _goal_node = _grid.update_goal_heading(goal, start);
    _astar.update_goal_node(*(_goal_node._base_node));
}

template <typename T>
const std::vector<std::vector<T>>& HybridAStar<T>::get_obstacles() const
{
    return _astar.get_obstacles();
}

template <typename T>
std::pair<T, bool> HybridAStar<T>::find_path(const T vel_init, const Vector3D<T>& start, 
        std::vector<Vector3D<T>>& path, std::vector<T>& curvature)
{
    // update goal and start locations
    Node3D<T> start_node = update_start(start);
    start_node._vmin_sqr = vel_init * vel_init;
    start_node._cost_f = std::numeric_limits<T>::max();
    Vector3D<T> goal_grid = _goal_node._pose2D;

    // find path using Hybrid A*
    _terminal_node = _goal_node;
    std::pair<T, bool> cost_success_pair = hybrid_a_star_search(start_node);

    // reconstruct path from goal to start if successful
    if (cost_success_pair.second)
    {
        reconstruct_path(_grid.get_goal_location(), goal_grid, path, curvature);
    }

    return cost_success_pair;
}


// Private member functions
template <typename T>
std::pair<T, bool> HybridAStar<T>::hybrid_a_star_search(Node3D<T>& start_node)
{
    // Hybrid A* initialization
    int dubins_shot_counter = 0;
    int dubins_shot_current_interval = _dubins_shot_interval;
    constexpr int dubins_shot_min_interval = 200;
    bool dubins_shot_allowed = false;
    _dubins_shot_successful = false;
    _closed_set.clear();
    _open_set.clear();
    _open_set.insert(start_node);
    std::vector<Node3D<T>> neighbors;

    // begin search till goal or intermediate goal if no path to original goal exists
    while (!_open_set.empty())
    {
        // remove first node from open set and add to closed set (lowest total cost)
        auto it = _open_set.begin();
        auto it_first = _closed_set.insert(*it).first;
        _open_set.erase(it); 

        // check if goal has been reached
        if (*it_first == _goal_node)
        {
            // copy actual goal node for path reconstruction
            _terminal_node = *it_first;

            return std::pair<T, bool>(_terminal_node._cost_g, true);
        }
        // check if a dubins shot should be attempted
        else if (dubins_shot_allowed)
        {
            dubins_shot_counter++;

            if (dubins_shot_counter == dubins_shot_current_interval)
            {
                std::pair<T, bool> path_info = _dubins.get_shortest_path(it_first->_pose2D, _goal_node._pose2D, 
                        _dubins_path, _dubins_abs_curvatures);
                
                // if successful then modify terminal node to be last node before dubins
                // shot and terminate search
                // boolean flag returned says whether the path initially includes a turn greater than 90 degs or not
                if ((!path_info.second) && _grid.check_path(_dubins_path))
                {
                    _terminal_node = *(it_first->_prev);
                    _dubins_shot_successful = true;

                    // std::cout << "Dubins shot successful \n"; 

                    return std::pair<T, bool>(it_first->_cost_g + path_info.first, true);
                }
                // update dubins shot interval
                else
                {
                    dubins_shot_counter = 0;
                    dubins_shot_current_interval = std::max(dubins_shot_current_interval - 
                            _dubins_shot_interval_decay, dubins_shot_min_interval);
                    
                    // std::cout << "Dubins shot failed \n"; 
                }
            }
        }
        
        // expand neighbors and add valid ones to open set
        dubins_shot_allowed = _grid.get_neighbors(*it_first, neighbors);
        // _grid.get_neighbors(*it_first, neighbors);
        for (auto& node : neighbors)
        {
            // handle node if not in the closed set 
            if (_closed_set.find(node) == _closed_set.end())
            {
                // add if not in open set
                auto it_node = _open_set.find(node);
                if (it_node == _open_set.end())
                {
                    // update both heuristics of Hybrid A* and choose the max from them
                    T holonomic_with_obstacles = _astar.find_path(node._base_node->_posd._x, 
                            node._base_node->_posd._y);
                    T nonholonomic_wo_obstacles = _dubins.get_shortest_path_length(node._pose2D, 
                            _goal_node._pose2D);
                    node._cost_f += std::max(holonomic_with_obstacles, nonholonomic_wo_obstacles);
                    node._prev = &(*it_first);
                    _open_set.insert(node);
                }
                // check if already in open set and swap if better path was found
                else if (node._cost_g < it_node->_cost_g)
                {
                    // remove old node
                    _open_set.erase(it_node);

                    // update and add new node
                    T holonomic_with_obstacles = _astar.find_path(node._base_node->_posd._x, 
                            node._base_node->_posd._y);
                    T nonholonomic_wo_obstacles = _dubins.get_shortest_path_length(node._pose2D, 
                            _goal_node._pose2D);
                    node._cost_f += std::max(holonomic_with_obstacles, nonholonomic_wo_obstacles);
                    node._prev = &(*it_first);
                    _open_set.insert(node);
                }
            }
        }
    }

    // find alternative goal and return false and max cost
    // choose_alternative_goal();
    return std::pair<T, bool>(std::numeric_limits<T>::max(), false);
}

template <typename T>
Node3D<T> HybridAStar<T>::update_start(const Vector3D<T>& start)
{
    return _grid.set_start_node(start);
}

template <typename T>
void HybridAStar<T>::reconstruct_path(const Vector3D<T>& goal, const Vector3D<T>& goal_grid, 
        std::vector<Vector3D<T>>& path, std::vector<T>& curvature) const
{
    T grid_heading = _grid.get_grid_heading();
    curvature.push_back(static_cast<T>(0)); // added to have equal sizes for path and curvature
    
    // copy dubins path in reverse order if successful
    if (_dubins_shot_successful)
    {
        std::size_t dubins_path_size = _dubins_path.size();
        path.resize(dubins_path_size);
        curvature.resize(dubins_path_size + 1);

        for (std::size_t i = 0; i < dubins_path_size; i++)
        {
            std::size_t path_index = dubins_path_size - i - 1;

            // get relative position to goal and rotate back to inertial frame
            Vector3D<T> pose2D(_dubins_path[path_index]._x - goal_grid._x,
                        _dubins_path[path_index]._y - goal_grid._y,
                        _dubins_path[path_index]._heading);
            pose2D = pose2D.get_rotated_vector(-grid_heading);

            // add goal offset to get absolute positon
            pose2D._x += goal._x;
            pose2D._y += goal._y;
            path[i] = pose2D;
            curvature[i + 1] = _dubins_abs_curvatures[path_index];
        }
    }

    // add the rest of the path
    const std::vector<T>& abs_curvatures = _grid.get_abs_curvatures();
    const Node3D<T>* current_node_ptr = &_terminal_node;
    while (current_node_ptr != nullptr)
    {   
        // get relative position to goal and rotate back to inertial frame
        Vector3D<T> pose2D(current_node_ptr->_pose2D._x - goal_grid._x,
                    current_node_ptr->_pose2D._y - goal_grid._y,
                    current_node_ptr->_pose2D._heading);
        pose2D = pose2D.get_rotated_vector(-grid_heading);

        // add goal offset to get absolute positon
        pose2D._x += goal._x;
        pose2D._y += goal._y;

        path.push_back(pose2D);
        curvature.push_back(abs_curvatures[current_node_ptr->_curvature_index]);
        current_node_ptr = current_node_ptr->_prev;
    }

    // remove curvature of first point, effectively shifting curvatures by 1 index backwards
    // so that each point's curvature represents that of the action taken at that point
    curvature.pop_back();
}

// Called in case the search failed to find a feasible path to the original goal
template <typename T>
void HybridAStar<T>::choose_alternative_goal()
{
    // loop over closed set and choose the best node as a new temporary goal
    int nodes_checked = 0;
    constexpr int max_nodes_checked = 2000;
    auto it_best = _closed_set.begin();
    for (auto it = it_best; (it != _closed_set.end()) && (nodes_checked < max_nodes_checked); it++)
    {
        nodes_checked++;
        if (it->_cost_f < it_best->_cost_f)
        {
            it_best = it;
        }
    }

    _terminal_node = *it_best;
}

// Explicit instantiation of supported types
template class HybridAStar<float>;
template class HybridAStar<double>;