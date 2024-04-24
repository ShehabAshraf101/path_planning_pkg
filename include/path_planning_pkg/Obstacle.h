#ifndef OBSTACLE
#define OBSTACLE

#include <cmath>
#include "common.h"

namespace planning
{
    template <typename T>
    struct Obstacle
    {
    
        // Struct members
        Vector3D<T> _pose2D;         // Obstacle postion (center) along the x and y axes and heading direction w.r.t. x-axis
        Vector2D<T> _velocity;       // Obstacle velocity along the x and y axes
        Vector2D<T> _dimensions;     // Obstacle dimensions along the x and y axes
    
        // Constructors
        Obstacle(T position_x, T position_y, T velocity_x, T velocity_y, T dimension_x, T dimension_y);
        Obstacle(T position_x, T position_y, T dimension_x, T dimension_y);
    };
}

#endif