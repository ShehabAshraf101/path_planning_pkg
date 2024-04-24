#ifndef VELOCITY_GENERATOR
#define VELOCITY_GENERATOR

#include <vector>
#include "common.h"

namespace planning
{
    template <typename T>
    class VelocityGenerator
    {
    
    public:
        // Contructors
        VelocityGenerator(T max_velocity, T coast_velocity, T max_lat_acc, T max_long_acc, T max_long_dec);

        // Public member fuctions
        bool generate_velocity_profile(const T vel_init, const std::vector<Vector3D<T>>& path, 
                const std::vector<T>& curvature, std::vector<T>& velocity, bool coast_to_goal, 
                bool stop_at_goal=false) const;
    private:
        // Private class members
        const T _max_velocity; // Maximum allowable longitudinal velocity (m/s)
        const T _coast_velocity; // Constant velocity to reach and maintain if no obstacle-free path was found
        const T _max_lat_acc; // Maximum allowable lateral acceleration (m/s^2)
        const T _max_lat_acc_sqr; // Maximum allowable lateral acceleration squared
        const T _max_long_acc; // Maximum allowable longitudinal acceleration during acceleration (m/s^2)
        const T _max_long_dec; // Maximum allowable longitudinal acceleration during braking (m/s^2)
    };
}

#endif