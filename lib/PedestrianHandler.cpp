#include "PedestrianHandler.h"

using namespace planning;

// Constructors
template <typename T>
PedestrianHandler<T>::PedestrianHandler(T detection_arc_angle, T min_stop_dist, 
        T min_allowable_ttc, T max_long_dec, T min_vel) :
        _detection_arc_angle_2(detection_arc_angle/2),
        _min_stop_dist(min_stop_dist),
        _min_allowable_ttc(min_allowable_ttc),
        _max_long_dec(max_long_dec),
        _min_vel(min_vel) {}


// Public member functions
template <typename T>
T PedestrianHandler<T>::calc_max_velocity(const T vel_curr, 
        const Vector3D<T>& pose_curr, const std::vector<Obstacle<T>>& pedestrians) const
{
    // find the pedestrain with the smallest ttc
    T min_ttc = std::numeric_limits<T>::max();
    Obstacle<T> pedestrain_closest(0, 0, 0, 0);
    for (const auto& pedestrian : pedestrians)
    {
        T pedestrian_ttc = calc_ttc(vel_curr, pose_curr, pedestrian);
        if (pedestrian_ttc < min_ttc)
        {
            min_ttc = pedestrian_ttc;
            pedestrain_closest = pedestrian;
        }
    }

    // calculate a max velocity if min_ttc < min_allowable_ttc
    if (min_ttc < _min_allowable_ttc)
    {
        T rel_angle_min, long_dist_min;
        calc_rel_angle_and_long_dist(pose_curr, pedestrain_closest, rel_angle_min, long_dist_min);

        // stop if closer than minimum allowable distance
        if (long_dist_min < _min_stop_dist)
        {
            return 0;
        }

        // return new max velocity based on min allowable ttc
        return std::max(
            (2 * long_dist_min + _max_long_dec * _min_allowable_ttc * _min_allowable_ttc)/
            (2 * _min_allowable_ttc), _min_vel);
    }
    
    return std::numeric_limits<T>::max();
}

// Private member functions
template <typename T>
T PedestrianHandler<T>::calc_ttc(const T vel_curr, const Vector3D<T>& pose_curr, 
        const Obstacle<T>& pedestrian) const
{
    // get angle of distance vector rel to vehicle orientation and longitudinal distance
    T rel_angle, long_dist;
    calc_rel_angle_and_long_dist(pose_curr, pedestrian, rel_angle, long_dist);
    
    // pedestrain is outside detection zone
    if (std::abs(rel_angle) > _detection_arc_angle_2)
    {
        return std::numeric_limits<T>::max();
    }

    // ttc is infinite (vehicle could stop before reaching)
    if ((2 * _max_long_dec * long_dist) > (vel_curr * vel_curr))
    {
        return std::numeric_limits<T>::max();
    }

    return (-vel_curr + std::sqrt(-2 * _max_long_dec * long_dist + vel_curr * vel_curr))/(-_max_long_dec);
}

template <typename T>
void PedestrianHandler<T>::calc_rel_angle_and_long_dist(const Vector3D<T>& pose_curr,
        const Obstacle<T>& pedestrian, T& rel_angle, T& long_dist) const
{
    rel_angle =  wrap_pi(std::atan2(pedestrian._pose2D._y - pose_curr._y,
            pedestrian._pose2D._x - pose_curr._x) - pose_curr._heading);
    long_dist = std::hypot(pedestrian._pose2D._x - pose_curr._x, 
            pedestrian._pose2D._y - pose_curr._y) * std::cos(rel_angle);
}        

// Explicit instantiation of supported types
template class PedestrianHandler<float>;
template class PedestrianHandler<double>;