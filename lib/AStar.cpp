#include "AStar.h"

using namespace planning;

// Constructors
#ifndef STORE_GRID_AS_REFERENCE
template <typename T>
AStar<T>::AStar(T grid_resolution, T obstacle_threshold, T obstacle_prob_min, T obstacle_prob_max, 
        T obstacle_prob_free, int grid_size, bool grid_allow_diag_moves) :
        _goal_node(0, 0), 
        _grid(Grid2D<T>(grid_resolution, obstacle_threshold, obstacle_prob_min, obstacle_prob_max, 
                obstacle_prob_free, grid_size, grid_allow_diag_moves)),
        _visted(grid_size, std::vector<bool>(grid_size, false)) {}

#else
template <typename T>
AStar<T>::AStar(Grid2D<T>& grid) : 
        _goal_node(0, 0), _grid(grid),
        _visted(grid.get_grid_size(), std::vector<bool>(grid.get_grid_size(), false)) {}
#endif

// Public functions
template <typename T>
void AStar<T>::update_goal_node(const Node2D<T>& goal_node)
{
    _goal_node = goal_node;
}

template <typename T>
void AStar<T>::update_goal_start(const Vector2D<T>& goal, const Vector2D<T>& start, Node2D<T>& start_node)
{
    _goal_node = _grid.update_goal_heading(goal, start);
    start_node = _grid.set_start_node(start);
}

template <typename T>
void AStar<T>::update_obstacles(const std::vector<Obstacle<T>>& obstacles, const std::vector<T>& confidence)
{
    _grid.update_obstacles(obstacles, confidence);
}

template <typename T>
void AStar<T>::update_obstacles(const std::vector<std::pair<Vector2D<T>, Vector2D<T>>>& lines, 
        const std::vector<T>& confidence, const T line_width)
{
    _grid.update_obstacles(lines, confidence, line_width);
}

template <typename T>
void AStar<T>::update_obstacles()
{
    _grid.update_obstacles();
}

template <typename T>
void AStar<T>::reset()
{
    _grid.clear_obstacles();
    int grid_size = _grid.get_grid_size();
    std::fill(_visted.begin(), _visted.end(), std::vector<bool>(grid_size, false)); 
}

template <typename T>
const std::vector<std::vector<T>>& AStar<T>::get_obstacles() const
{
    return _grid.get_obstacle_map();
}

// Reconstructs path and returns actual cost to reach goal (cost = std::numeric_limit<T>::max() if no path is found)
// Note: reconstructed path is reversed starting at goal and ending at start (use reverse iterator)
template <typename T>
T AStar<T>::find_path(const Vector2D<T>& goal, const Vector2D<T>& start, std::vector<Vector2D<T>>& path)
{
    // compute accumulated cost of reaching goal from start using A*
    T accumulated_cost = find_path(goal, start, false);

    // reconstruct path if one exists
    if (accumulated_cost < std::numeric_limits<T>::max())
    {
        reconstruct_path(goal, path);
    }

    return accumulated_cost;
}

// Returns actual cost to reach goal only (cost = std::numeric_limit<T>::max() if no path is found)
template <typename T>
T AStar<T>::find_path(const Vector2D<T>& goal, const Vector2D<T>& start, bool get_cost_only)
{
    // update goal and start locations as well as obstacles
    _goal_node = _grid.update_goal_heading(goal, start);
    Node2D<T> start_node = _grid.set_start_node(start);

    // return accumulated cost of reaching goal from start found using A*
    return a_star_search(start_node, get_cost_only);
}

// returns actual cost to reach goal starting from grid cell (i, j) (goal and obstacles unchanged)
// (cost = std::numeric_limit<T>::max() if no path is found)
template <typename T>
T AStar<T>::find_path(const int start_i, const int start_j)
{
    // if start node has already been visited then return cost and terminate without searching
    if (_visted[start_i][start_j])
    {
        return _grid.get_node_total_cost(start_i, start_j);
    }

    // update start location on grid without changing goal or obstacle locations
    Node2D<T> start_node = _grid.set_start_node_grid(start_i, start_j);

    // return accumulated cost of reaching goal from start found using A*
    return a_star_search(start_node);
}


// Private functions
template <typename T>
T AStar<T>::a_star_search(Node2D<T>& start_node, bool get_cost_only)
{
    // A* initialization
    _closed_set.clear();
    _open_set.clear();
    _open_set.insert(start_node);
    std::vector<std::pair<Node2D<T>*, T>> neighbors;

    // begin search till goal is found or grid has been fully explored
    while (!_open_set.empty())
    {
        // remove first node from open set and add to closed set (lowest total cost)
        auto it = _open_set.begin();
        auto it_first = _closed_set.insert(*it).first;
        _open_set.erase(it);

        if (*it_first == _goal_node)
        {
            // store actual goal node for reconstructing path if needed
            _goal_node = *it_first;
            if (get_cost_only)
            {
                update_visted(_goal_node._cost_f, _goal_node);
            }

            return _goal_node._cost_f;
        }
        else
        {
            // expand neighbors and add valid ones to open set
            _grid.get_neighbors(it_first->_posd._x, it_first->_posd._y, neighbors);
            T cost_g_first = it_first->_cost_g;
            for (auto& pair : neighbors)
            {
                // check if a path to goal from neighbor already exists (only cost required)
                if (get_cost_only && _visted[(pair.first)->_posd._x][(pair.first)->_posd._y])
                {
                    T total_cost = (pair.first)->_cost_f + cost_g_first + pair.second;
                    update_visted(total_cost, *it_first);
                    
                    return total_cost; 
                }

                // handle node if not in the closed set 
                if (_closed_set.find(*(pair.first)) == _closed_set.end())
                {
                    // add if not in open set
                    auto it_node = _open_set.find(*(pair.first));
                    if (it_node == _open_set.end())
                    {
                        (pair.first)->set_accumulated_cost(cost_g_first + pair.second);
                        (pair.first)->_prev = &(*it_first);
                        _open_set.insert(*(pair.first));
                    }
                    // check if already in open set and swap if better path was found
                    else if ((cost_g_first + pair.second) < it_node->_cost_g)
                    {
                        _open_set.erase(it_node);
                        (pair.first)->set_accumulated_cost(cost_g_first + pair.second);
                        (pair.first)->_prev = &(*it_first);
                        _open_set.insert(*(pair.first));
                    }
                }
            }
        }
    }

    return std::numeric_limits<T>::max();
}

template <typename T>
void AStar<T>::reconstruct_path(const Vector2D<T>& goal, std::vector<Vector2D<T>>& path) const
{
    T grid_resolution = _grid.get_grid_resolution();
    T grid_heading = _grid.get_grid_heading();
    const Node2D<T>* current_node_ptr = _goal_node._prev;
    path.push_back(goal);
    // const Node2D<T>* current_node_ptr = &_goal_node;

    while (current_node_ptr != nullptr)
    {
        Vector2D<T> rel_pos((current_node_ptr->_posd._x - _goal_node._posd._x) * grid_resolution,
                    (current_node_ptr->_posd._y - _goal_node._posd._y) * grid_resolution);
        path.push_back(rel_pos.get_rotated_vector(-grid_heading) + goal);
        // Vector2D<T> pos(current_node_ptr->_posd._x, current_node_ptr->_posd._y);
        // path.push_back(pos);
        current_node_ptr = current_node_ptr->_prev;
    }
}

template <typename T>
void AStar<T>::update_visted(const T total_cost, const Node2D<T>& last_node)
{
    const Node2D<T>* current_node_ptr = &last_node;
    while (current_node_ptr != nullptr)
    {
        _visted[current_node_ptr->_posd._x][current_node_ptr->_posd._y] = true;
        current_node_ptr = current_node_ptr->_prev;
    }
    _grid.update_costs(total_cost, last_node);
}

// Explicit instantiation of supported types
template class AStar<float>;
template class AStar<double>;