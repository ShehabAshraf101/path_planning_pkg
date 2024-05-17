#include "Obstacle.h"

using namespace planning;

// Constructors
template <typename T>
Obstacle<T>::Obstacle(T position_x, T position_y, T velocity_x, T velocity_y, T dimension_x, T dimension_y) :
        _pose2D(position_x, position_y, static_cast<T>(0)), 
        _velocity(velocity_x, velocity_y), 
        _dimensions(dimension_x, dimension_y)
{
    // initialized after initializer list because velocity must be intialized first
    _pose2D._heading = std::atan2(_velocity._y, _velocity._x);
}

template <typename T>
Obstacle<T>::Obstacle(T position_x, T position_y, T dimension_x, T dimension_y) :
        Obstacle(position_x, position_y, static_cast<T>(0), static_cast<T>(0), 
                dimension_x, dimension_y) {}

// Explicit instantiation of supported types
template class Obstacle<float>;
template class Obstacle<double>;