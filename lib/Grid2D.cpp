#include "Grid2D.h"

using namespace planning;

// Constructors
template <typename T>
Grid2D<T>::Grid2D(T resolution, int grid_size, Vector2D<T> goal, Vector2D<T> start, bool allow_diag_moves) :
        _grid_heading(std::atan2(goal._y - start._y, goal._x - start._x)), _resolution(resolution),
        _grid_size(grid_size), _grid_size_2(static_cast<int>(std::round(grid_size * 0.5))),
        _grid_size_4_5(static_cast<int>(std::round(grid_size * 0.8))), _goal_location(goal),
        _node_map(grid_size, std::vector<Node2D<T>>(grid_size, Node2D<T>(0, 0))),
        _obstacle_map(grid_size, std::vector<bool>(grid_size, false))
{
    // loop over grid to initialize nodes
    for (int i = 0; i < _grid_size; i++)
    {
        for (int j = 0; j < _grid_size; j++)
        {
            _node_map[i][j] = Node2D<T>(i, j);
        }
    }

    // initialize actions based on allow_diag_moves (8 or 4 actions)
    if (allow_diag_moves)
    {
        _actions.resize(8);
        _actions = {{0,  -1},
                    {1,  -1},
                    {1,   0},
                    {1,   1},
                    {0,   1},
                    {-1,  1},
                    {-1,  0},
                    {-1, -1}};
    }
    else
    {
        _actions.resize(4);
        _actions = {{0, -1},
                    {1,  0},
                    {0,  1},
                    {-1, 0}};
    }

    // calculate the cost of each action based on Euclidean distance
    for (const auto& action : _actions)
    {
        T action_cost = _resolution * std::sqrt(static_cast<T>(action.first * action.first + action.second * action.second));
        _actions_cost.push_back(action_cost);
    }

    // pre-compute the heuristic for all cells except the goal
    compute_heuristic();
}

template <typename T>
Grid2D<T>::Grid2D(T resolution, int grid_size, bool allow_diag_moves) :
        Grid2D(resolution, grid_size, Vector2D<T>(), Vector2D<T>(), allow_diag_moves) {}

// Public functions
template <typename T>
void Grid2D<T>::get_neighbors(const int xd, const int yd, std::vector<std::pair<Node2D<T>*, T>>& neighbors)
{
    // apply each action to get neighbor if valid (in bounds and not obstacle)
    int valid_neighbors = 0;
    int num_actions = _actions.size();
    neighbors.resize(num_actions);
    for (int k = 0; k < num_actions; k++)
    {
        int i = xd + _actions[k].first;
        int j = yd + _actions[k].second;
        if ((i > -1) && (i < _grid_size) && (j > -1) && (j < _grid_size))
        {
            if (!_obstacle_map[i][j])
            {
                neighbors[valid_neighbors].first = &_node_map[i][j];
                neighbors[valid_neighbors].second = _actions_cost[k];
                valid_neighbors++;
            }
        }
    }

    if (valid_neighbors != num_actions)
    {
        neighbors.resize(valid_neighbors);
    }
}

template <typename T>
void Grid2D<T>::update_obstacles(const std::vector<Obstacle<T>>& obstacles)
{
    for (const auto& obstacle : obstacles)
    {
        constexpr int added_size_2 = 2;
        Vector2D<T> rel_position(obstacle._pose2D._x - obstacle._dimensions._x/2 - _goal_location._x, 
                obstacle._pose2D._y - obstacle._dimensions._y/2 - _goal_location._y);
        rel_position.rotate_vector(_grid_heading);
        int i_bl = static_cast<int>(rel_position._x/_resolution) + _grid_size_4_5 - added_size_2; // Index of bottom left corner
        int j_bl = static_cast<int>(rel_position._y/_resolution) + _grid_size_2 - added_size_2;  // Index of bottom left corner
        int end_i = static_cast<int>(std::ceil(obstacle._dimensions._x/_resolution)) + added_size_2;
        int end_j = static_cast<int>(std::ceil(obstacle._dimensions._y/_resolution)) + added_size_2;

        // discretize obstacles into square cells
        for (int i = 0; i < end_i; i++)
        {
            // T dx = i;
            for (int j = 0; j < end_j; j++)
            {
                Vector2D<T> offset(i, j);
                offset.rotate_vector(_grid_heading);
                int i_p = i_bl + static_cast<int>(std::round(offset._x));
                int j_p = j_bl + static_cast<int>(std::round(offset._y));
                if((i_p > -1) && (i_p < _grid_size) && (j_p > -1) && (j_p < _grid_size))
                {
                    _obstacle_map[i_p][j_p] = true;
                }
            }
        }
    }
}

// Erases the status of obstacles  
template <typename T>
void Grid2D<T>::clear_obstacles()
{
    // reset obstacle status
    for (auto& row : _obstacle_map)
    {
        std::fill(row.begin(), row.end(), false);
    }
}

template <typename T>
void Grid2D<T>::update_costs(const T total_cost, const Node2D<T>& last_node)
{
    const Node2D<T>* current_node_ptr = &last_node;
    while (current_node_ptr != nullptr)
    {
        _node_map[current_node_ptr->_posd._x][current_node_ptr->_posd._y]._cost_f = total_cost - current_node_ptr->_cost_g;
        current_node_ptr = current_node_ptr->_prev;
    }
}

template <typename T>
T Grid2D<T>::get_node_total_cost(const int i, const int j) const
{
    return _node_map[i][j]._cost_f;
}

template <typename T>
T Grid2D<T>::get_grid_heading() const
{
    return _grid_heading;
}

template <typename T>
T Grid2D<T>::get_grid_resolution() const
{
    return _resolution;
}

template <typename T>
int Grid2D<T>::get_grid_size() const
{
    return _grid_size;
}

template <typename T>
Node2D<T> Grid2D<T>::update_goal_heading(const Vector2D<T>& goal, const Vector2D<T>& start)
{
    _goal_location = goal;
    _grid_heading = std::atan2(goal._y - start._y, goal._x - start._x);

    return _node_map[_grid_size_4_5][_grid_size_2];
}

// Modifies start node location without changing goal or grid orientation (coordinates in inertial frame)
template <typename T>
Node2D<T> Grid2D<T>::set_start_node(const Vector2D<T>& start)
{
    // get relative position in the grid's frame
    Vector2D<T> rel_position = (start - _goal_location).get_rotated_vector(_grid_heading);

    // get discrete coordinates and add offset (goal positon)
    int i = static_cast<int>(rel_position._x/_resolution) + _grid_size_4_5;
    int j = static_cast<int>(rel_position._y/_resolution) + _grid_size_2;

    // check validity of start location
    if((i > -1) && (i < _grid_size) && (j > -1) && (j < _grid_size))
    {
        // reset the node's cost and predeccesor and return a copy of it
        _node_map[i][j].soft_reset();
        return _node_map[i][j];
    }

    // Default to (0, 0) node
    _node_map[0][0].soft_reset();
    return _node_map[0][0];
}

// Modifies start node location without changing goal or grid orientation (discrete coordinates given)
template <typename T>
Node2D<T> Grid2D<T>::set_start_node_grid(const int i, const int j)
{
    // reset the node's cost and predeccesor and return a copy of it
    _node_map[i][j].soft_reset();
    return _node_map[i][j];
}

// Protected functions
template <typename T>
void Grid2D<T>::compute_heuristic()
{
    for (int i = 0; i < _grid_size; i++)
    {
        T delta_x = (_grid_size_4_5 - i) * _resolution;
        T delta_x_sqr = delta_x * delta_x;
        for (int j = 0; j < _grid_size; j++)
        {
            T delta_y = (_grid_size_2 - j) * _resolution;
            T delta_y_sqr = delta_y * delta_y;
            _node_map[i][j].set_heuristic_cost(std::sqrt(delta_x_sqr + delta_y_sqr));
        }
    }
}

// Explicit instantiation of supported types
template class Grid2D<float>;
template class Grid2D<double>;