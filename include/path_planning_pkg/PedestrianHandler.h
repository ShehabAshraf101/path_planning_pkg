#ifndef PEDESTRIAN_HANDLER
#define PEDESTRIAN_HANDLER

#include <vector>
#include <limits>
#include "Obstacle.h"
#include "common.h"

namespace planning
{
    template <typename T>
    class PedestrianHandler
    {

    public:
        // Constructors
        PedestrianHandler(T detection_arc_angle, T min_stop_dist, T min_allowable_ttc, 
                T max_long_dec, T min_vel);

        // Public member functions
        T calc_max_velocity(const T vel_curr, const Vector3D<T>& pose_curr, 
                const std::vector<Obstacle<T>>& pedestrians) const;

    private:
        // Private member functions
        T calc_ttc(const T vel_curr, const Vector3D<T>& pose_curr, const Obstacle<T>& pedestrian) const;
        void calc_rel_angle_and_long_dist(const Vector3D<T>& pose_curr, const Obstacle<T>& pedestrian,
                T& rel_angle, T& long_dist) const;
        
        // Private class members
        const T _detection_arc_angle_2;           // Arc angle/2 relative to the vehicle's heading within which pedestrains are detected
        const T _min_stop_dist;                   // Minimum distance to pedestrians, below which the vehicle should come to a halt
        const T _min_allowable_ttc;               // Minimum allowable time to collision (TTC) for distances > min_stop_dist
        const T _max_long_dec;                    // Maximum allowable deceleration (m/s^2) for TTC calculation
        const T _min_vel;                         // Minimum velocity threshold for calculating max velocity
    };
}

#endif