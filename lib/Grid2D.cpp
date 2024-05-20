#include "Grid2D.h"

using namespace planning;

// Constructors
template <typename T>
Grid2D<T>::Grid2D(T resolution, T obstacle_threshold, T obstacle_prob_min, T obstacle_prob_max, 
        T obstacle_prob_free, int grid_size, Vector2D<T> goal, Vector2D<T> start, bool allow_diag_moves) :
        _grid_heading(std::atan2(goal._y - start._y, goal._x - start._x)), 
        _resolution(resolution),
        _obstacle_log_threshold(std::log(obstacle_threshold/(1.0 - obstacle_threshold))),
        _obstacle_log_prob_min(std::log(obstacle_prob_min/(1.0 - obstacle_prob_min))),
        _obstacle_log_prob_max(std::log(obstacle_prob_max/(1.0 - obstacle_prob_max))),
        _obstacle_log_prob_free(std::log(obstacle_prob_free/(1.0 - obstacle_prob_free))),
        _grid_size(grid_size), 
        _grid_size_2(static_cast<int>(std::round(grid_size * 0.5))),
        _grid_size_4_5(static_cast<int>(std::round(grid_size * 0.8))), 
        _goal_location(goal),
        _node_map(grid_size, std::vector<Node2D<T>>(grid_size, Node2D<T>(0, 0))),
        _obstacle_map(grid_size, std::vector<T>(grid_size, static_cast<T>(0.0)))
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
Grid2D<T>::Grid2D(T resolution, T obstacle_threshold, T obstacle_log_prob_min, T obstacle_log_prob_max,
        T obstacle_prob_free, int grid_size, bool allow_diag_moves) :
        Grid2D(resolution, obstacle_threshold, obstacle_log_prob_min, obstacle_log_prob_max, 
                obstacle_prob_free, grid_size, Vector2D<T>(), Vector2D<T>(), allow_diag_moves) {}

// Public functions
template <typename T>
void Grid2D<T>::get_neighbors(const int xd, const int yd, std::vector<std::pair<Node2D<T>*, T>>& neighbors)
{
    // apply each action to get neighbor if valid (in bounds and not obstacle)
    std::size_t valid_neighbors = 0;
    neighbors.resize(_actions.size());
    for (std::size_t k = 0; k < _actions.size(); k++)
    {
        int i = xd + _actions[k].first;
        int j = yd + _actions[k].second;
        if ((i > -1) && (i < _grid_size) && (j > -1) && (j < _grid_size))
        {
            if (_obstacle_map[i][j] < _obstacle_log_threshold)
            {
                neighbors[valid_neighbors].first = &_node_map[i][j];
                neighbors[valid_neighbors].second = _actions_cost[k];
                valid_neighbors++;
            }
        }
    }

    if (valid_neighbors != _actions.size())
    {
        neighbors.resize(valid_neighbors);
    }
}

template <typename T>
void Grid2D<T>::update_obstacles(const std::vector<Obstacle<T>>& obstacles, const std::vector<T>& confidence)
{
    // define a lambda function to get the indices of a location on the map
    auto get_indices = [this] (const T& position_x, const T& position_y) -> std::pair<int, int>
    {
        Vector2D<T> position(position_x - _goal_location._x, position_y - _goal_location._y);
        position.rotate_vector(_grid_heading);
        
        return std::pair<int, int>(static_cast<int>(std::round(position._x/_resolution) + _grid_size_4_5),
                static_cast<int>(std::round(position._y/_resolution) + _grid_size_2));
    };

    for (std::size_t k = 0; k < obstacles.size(); k++)
    { 
        int start_i, end_i, start_j, end_j;
        std::pair<int, int> bl_indices = get_indices(obstacles[k]._pose2D._x - obstacles[k]._dimensions._x/2, 
                obstacles[k]._pose2D._y - obstacles[k]._dimensions._y/2);
        start_i = bl_indices.first;
        start_j = bl_indices.second; 
        end_i = static_cast<int>(std::ceil(obstacles[k]._dimensions._x/_resolution));
        end_j = static_cast<int>(std::ceil(obstacles[k]._dimensions._y/_resolution));

        // discretize obstacles into square cells
        T log_confidence = std::log(confidence[k]/(1.0 - confidence[k]));
        for (int i = 0; i < 2 * end_i; i++)
        {
            for (int j = 0; j < 2 * end_j; j++)
            {
                Vector2D<T> offset(i * 0.5, j * 0.5);
                offset.rotate_vector(_grid_heading);
                int i_p = start_i + static_cast<int>(std::round(offset._x));
                int j_p = start_j + static_cast<int>(std::round(offset._y));
                if((i_p > -1) && (i_p < _grid_size) && (j_p > -1) && (j_p < _grid_size))
                {
                    _obstacle_map[i_p][j_p] += log_confidence - _obstacle_log_prob_free;
                    _obstacle_map[i_p][j_p] = std::max(std::min(_obstacle_map[i_p][j_p], _obstacle_log_prob_max), _obstacle_log_prob_min);
                }
            }
        }
    }
}

template <typename T>
void Grid2D<T>::update_obstacles(const std::vector<std::pair<Vector2D<T>, Vector2D<T>>>& lines, 
        const std::vector<T>& confidence, const T line_width)
{
    for (std::size_t k = 0; k < lines.size(); k++)
    {
        Vector2D<T> start_point = (lines[k].first - _goal_location).get_rotated_vector(_grid_heading);
        Vector2D<T> end_point = (lines[k].second - _goal_location).get_rotated_vector(_grid_heading);
        Vector2D<T> delta(end_point._x - start_point._x, end_point._y - start_point._y);
        T line_length = std::hypot(delta._x, delta._y);
        Vector2D<T> delta_normal = Vector2D<T>(-delta._y, delta._x)/line_length; 
        T prog_length = 0;
        
        // discretize line along its length
        T log_confidence = std::log(confidence[k]/(1.0 - confidence[k]));
        constexpr std::size_t max_iterations = 100;
        std::size_t iter_count = 0;
        while (prog_length <= 1.0 && (iter_count < max_iterations))
        {
            // discretize line along its width
            T prog_width = 0;
            Vector2D<T> intercept = start_point + delta * prog_length;
            while (prog_width <= line_width)
            {
                Vector2D<T> point1 = intercept + delta_normal * prog_width;
                Vector2D<T> point2 = intercept - delta_normal * prog_width;
                int i1 = static_cast<int>(std::round(point1._x/_resolution)) + _grid_size_4_5;
                int i2 = static_cast<int>(std::round(point2._x/_resolution)) + _grid_size_4_5;
                int j1 = static_cast<int>(std::round(point1._y/_resolution)) + _grid_size_2;
                int j2 = static_cast<int>(std::round(point2._y/_resolution)) + _grid_size_2;

                // update points if inside grid
                if((i1 > -1) && (i1 < _grid_size) && (j1 > -1) && (j1 < _grid_size))
                {
                    _obstacle_map[i1][j1] += log_confidence - _obstacle_log_prob_free;
                    _obstacle_map[i1][j1] = std::max(std::min(_obstacle_map[i1][j1], _obstacle_log_prob_max), _obstacle_log_prob_min);
                }
                if((i2 > -1) && (i2 < _grid_size) && (j2 > -1) && (j2 < _grid_size))
                {
                    _obstacle_map[i2][j2] += log_confidence - _obstacle_log_prob_free;
                    _obstacle_map[i2][j2] = std::max(std::min(_obstacle_map[i2][j2], _obstacle_log_prob_max), _obstacle_log_prob_min);
                }

                // update progess along width
                prog_width += _resolution;
            }

            // update progess along length
            prog_length += _resolution/line_length;
            iter_count++;
        }
    }
}

template <typename T>
void Grid2D<T>::update_obstacles()
{
    // update the status of all obstacles as being free cells
    std::for_each(_obstacle_map.begin(), _obstacle_map.end(), [this] (std::vector<T>& row)
    {
        std::for_each(row.begin(), row.end(), [this] (T& log_probability)
        {
            log_probability += _obstacle_log_prob_free;
            log_probability = std::max(std::min(log_probability, _obstacle_log_prob_max), _obstacle_log_prob_min);
        });
    });
}

// Erases the status of obstacles  
template <typename T>
void Grid2D<T>::clear_obstacles()
{
    // reset obstacle status
    std::fill(_obstacle_map.begin(), _obstacle_map.end(), std::vector<T>(_grid_size, 0.0));
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
const std::vector<std::vector<T>>& Grid2D<T>::get_obstacle_map() const
{
    return _obstacle_map;
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