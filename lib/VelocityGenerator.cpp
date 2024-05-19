#include "VelocityGenerator.h"

using namespace planning;

template <typename T>
// Contructors
VelocityGenerator<T>::VelocityGenerator(T max_velocity, T coast_velocity, T max_lat_acc, 
        T max_long_acc, T max_long_dec) :
        _max_velocity(max_velocity), 
        _coast_velocity(coast_velocity), 
        _max_lat_acc(max_lat_acc), 
        _max_lat_acc_sqr(max_lat_acc * max_lat_acc),
        _max_long_acc(max_long_acc), 
        _max_long_dec(max_long_dec) {}


// Public member fuctions
template <typename T>
bool VelocityGenerator<T>::generate_velocity_profile(const T vel_init, const std::vector<Vector3D<T>>& path, 
        const std::vector<T>& curvature, std::vector<T>& velocity, bool coast_to_goal, bool stop_at_goal) const
{
    // set max velocity based on coast_to_goal flag
    const T max_velocity = (coast_to_goal) ? _coast_velocity : _max_velocity;
    const T max_velocity_sqr = max_velocity * max_velocity;

    // resize velocity vector to the same size as path
    std::size_t path_size = path.size();
    velocity.resize(path_size);
    std::vector<T> velocity_sqr(path_size); 

    // calculate initial velocity profile (path and curvature are reversed (goal -> start))
    for (std::size_t i = 0; i < path_size; i++)
    {
        std::size_t path_index = path_size - i - 1;
        
        velocity_sqr[i] = (curvature[path_index] != 0) ? std::min(_max_lat_acc/curvature[path_index], max_velocity_sqr) : 
                max_velocity_sqr;
    }

    // set velocities of endpoints
    velocity_sqr[0] = vel_init * vel_init;
    velocity_sqr[path_size - 1] = (stop_at_goal) ? 0 : velocity_sqr[path_size - 1];

    // perform forward pass on velocity profile
    for (std::size_t i = 0; i < (path_size - 1); i++)
    {
        std::size_t path_index = path_size - i - 1;

        T dx = path[path_index - 1]._x - path[path_index]._x;
        T dy = path[path_index - 1]._y - path[path_index]._y;
        T step_size = std::sqrt(dx * dx + dy * dy);

        T lat_acc = velocity_sqr[i] * curvature[path_index];
        T long_acc_rem = _max_long_acc * std::sqrt(1.0 - (lat_acc * lat_acc)/_max_lat_acc_sqr);
        velocity_sqr[i + 1] = std::min(velocity_sqr[i] + 2 * long_acc_rem * step_size, velocity_sqr[i + 1]);
    }

    // perform backward pass on velocity profile
    for (std::size_t i = path_size - 1; i > 0; i--)
    {
        std::size_t path_index = path_size - i - 1;

        T dx = path[path_index + 1]._x - path[path_index]._x;
        T dy = path[path_index + 1]._y - path[path_index]._y;
        T step_size = std::sqrt(dx * dx + dy * dy);

        T lat_acc = velocity_sqr[i] * curvature[path_index];
        T long_acc_rem = _max_long_dec * std::sqrt(1.0 - (lat_acc * lat_acc)/_max_lat_acc_sqr);
        velocity_sqr[i - 1] = std::min(velocity_sqr[i] + 2 * long_acc_rem * step_size, velocity_sqr[i - 1]);
        velocity[i - 1] = std::sqrt(velocity_sqr[i - 1]);
    }

    // calculate the velocity of endpoint and return true if vel_init < (v[0) + tolerance) --> (feasible motion)
    velocity[path_size - 1] = std::sqrt(velocity_sqr[path_size - 1]);
    constexpr T velocity_tolerance = static_cast<T>(0.25);
    bool is_feasible = (vel_init < (velocity[0] + velocity_tolerance));

    return is_feasible;
}

// Explicit instantiation of supported types
template class VelocityGenerator<float>;
template class VelocityGenerator<double>;