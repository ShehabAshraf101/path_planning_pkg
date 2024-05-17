#include "VehicleModel.h"

using namespace planning;

// Constructors
template <typename T>
VehicleModel<T>::VehicleModel(T ts, T max_lat_acc, T max_long_dec, T wheelbase, T rear_to_cg, int num_angle_bins, 
        int num_actions, const std::vector<T>& steering, const std::vector<T>& curvature_weights) :
        _ts(ts), 
        _max_lat_acc(max_lat_acc), 
        _max_lat_acc_sqr(max_lat_acc * max_lat_acc), 
        _max_long_dec(max_long_dec), 
        _precision(2 * M_PI/num_angle_bins), 
        _num_actions(num_actions), 
        _abs_curvatures(steering.size()), 
        _actions_cost(steering.size()), 
        _offset_heading(steering.size()), 
        _offset_xy(steering.size(), std::vector<Vector2D<T>>(num_angle_bins, Vector2D<T>()))
{
    // calculate sideslip angle (beta) and curvature (signed) for each steering input
    std::vector<T> beta(steering.size());
    for (size_t i = 0; i < steering.size(); i++)
    {
        beta[i] = std::atan2(rear_to_cg * std::tan(steering[i]), wheelbase);
        _abs_curvatures[i] = std::cos(beta[i]) * std::tan(steering[i])/wheelbase;
        // std::cout << _abs_curvatures[i] << ", ";
    }
    // std::cout << "\n";

    // calculate the offset vector for each steering and orientation pair
    for (size_t i = 0; i < steering.size(); i++)
    {
        _offset_heading[i] = _ts * _abs_curvatures[i];
        _actions_cost[i] = _ts + curvature_weights[i] * std::abs(_abs_curvatures[i]);
        for (int j = 0; j < num_angle_bins; j++)
        {   
            Vector2D<T> offset_xy = calculate_offset(beta[i], _abs_curvatures[i], -M_PI + j * _precision);
            _offset_xy[i][j] = offset_xy;
        }
    }

    // take the absolute value of all curvatures since the sign is no longer needed
    for (auto& curvature : _abs_curvatures)
    {
        curvature = std::abs(curvature);
    }
}

// Public member functions
template <typename T>
T VehicleModel<T>::get_precision() const
{
    return _precision;
}

template <typename T>
int VehicleModel<T>::get_default_action_index() const
{
    return static_cast<int>(_abs_curvatures.size()/2);
}

template <typename T>
bool VehicleModel<T>::get_neighbors(const Node3D<T>& node, std::vector<Node3D<T>>& neighbors) const
{
    // loop over all possible actions and generate feasible neighbors from them
    int size = _abs_curvatures.size();
    int start_index = node._curvature_index - _num_actions;
    start_index = (start_index < 0) ? 0 : start_index;

    int num_neighbors = _num_actions * 2 + 1;
    neighbors.reserve(num_neighbors);
    neighbors.clear();

    constexpr T vmin_sqr_lower_threshold = static_cast<T>(1.0); // below which accelerations are neglected
    bool neglect_acceleration = (node._vmin_sqr < vmin_sqr_lower_threshold);
    for (int i = start_index; (i < (start_index + num_neighbors)) && (i < size); i++)
    {
        // consider accelerations if velocity above threshold
        T vmin_sqr = 0;
        if (!neglect_acceleration)
        {
            // check that lateral acceleration doesn't exceed limit
            T min_lat_acc = node._vmin_sqr * _abs_curvatures[i];
            if (min_lat_acc > _max_lat_acc)
            {
                continue;
            }
            else
            {
                // calculate velocity squared of next node based on chosen action
                T acc_long = std::sqrt(1.0 - ((min_lat_acc * min_lat_acc)/_max_lat_acc_sqr));
                vmin_sqr = node._vmin_sqr - 2 * acc_long * _ts;
            }
        }

        // initialize a new node and add to neighbors
        Vector3D<T> pose2D(node._pose2D._x + _offset_xy[i][node._angle_bin]._x,
                    node._pose2D._y + _offset_xy[i][node._angle_bin]._y,
                    wrap_pi(node._pose2D._heading + _offset_heading[i]));
        neighbors.emplace_back(pose2D, node._cost_g + _actions_cost[i], vmin_sqr, i, 
                            get_heading_index(pose2D._heading, _precision), &node);
    }

    return neglect_acceleration;
}

template <typename T>
std::pair<bool, Node3D<T>> VehicleModel<T>::simulate_action(const Node3D<T>& node, const int action_index) const
{
    // consider accelerations if velocity is still high
    T vmin_sqr = static_cast<T>(0);
    if (node._vmin_sqr > 1.0)
    {
        // check that lateral acceleration doesn't exceed limit
        T min_lat_acc = node._vmin_sqr * _abs_curvatures[action_index];
        if (min_lat_acc > _max_lat_acc)
        {
            Vector3D<T> pose2D;
            return std::pair<bool, Node3D<T>>(false, node);
        }
        else
        {
            // calculate velocity squared of next node based on chosen action
            T acc_long = std::sqrt(1.0 - ((min_lat_acc * min_lat_acc)/_max_lat_acc_sqr));
            vmin_sqr = node._vmin_sqr - 2 * acc_long * _ts;
        }
    }
    // initialize a new node and add to neighbors
    Vector3D<T> pose2D(node._pose2D._x + _offset_xy[action_index][node._angle_bin]._x,
                node._pose2D._y + _offset_xy[action_index][node._angle_bin]._y,
                wrap_pi(node._pose2D._heading + _offset_heading[action_index]));

    return std::pair<bool, Node3D<T>>(std::piecewise_construct, std::forward_as_tuple(true), 
            std::forward_as_tuple(pose2D, node._cost_g + _actions_cost[action_index], vmin_sqr, action_index,
            get_heading_index(pose2D._heading, _precision), &node));
}

template <typename T>
const std::vector<T>& VehicleModel<T>::get_abs_curvatures() const
{
    return _abs_curvatures;
}


// Private member functions
template <typename T>
Vector2D<T> VehicleModel<T>::calculate_offset(const T beta, const T curvature, 
        const T heading) const
{
    constexpr T dt = static_cast<T>(0.001);
    T offset_x = static_cast<T>(0);
    T offset_y = static_cast<T>(0);
    T curr_heading = heading;

    int num_updates = static_cast<int>(_ts/dt);
    for (int i = 0; i < num_updates; i++)
    {
        offset_x += dt * std::cos(beta + curr_heading);
        offset_y += dt * std::sin(beta + curr_heading);
        curr_heading += dt * curvature;
    }

    return Vector2D<T>(offset_x, offset_y);
}

// Explicit instantiation of supported types
template class VehicleModel<float>;
template class VehicleModel<double>;