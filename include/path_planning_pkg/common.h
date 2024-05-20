#ifndef PLANNING_COMMON
#define PLANNING_COMMON

#include <cmath>

namespace planning
{
    template <typename T>
    T round_to_nearest(const T value, const T precision)
    {
        return std::round(value/precision) * precision;
    }

    template <typename T>
    T wrap_pi(const T angle)
    {
        T wrap_2pi = std::fmod(angle, 2 * M_PI);
        if (wrap_2pi > M_PI)
        {
            return wrap_2pi - 2 * M_PI;
        }

        if (wrap_2pi < -M_PI)
        {
            return wrap_2pi + 2 * M_PI;
        }
        
        return wrap_2pi;
    }

    template <typename T>
    int get_heading_index(const T heading, const T precision)
    {
        T rounded_heading = round_to_nearest(heading, precision);
        return static_cast<int>((rounded_heading + M_PI)/precision);
    }

    template <typename T> 
    struct Vector2D 
    {

        // Struct members
        T _x;    // X coordinate
        T _y;    // Y coordinate

        // Constructors
        Vector2D() : _x(static_cast<T>(0)), _y(static_cast<T>(0)) {}
        Vector2D(T x, T y) : _x(x), _y(y) {}

        // Functions
        Vector2D<T> get_rotated_vector(const T angle) const
        {
            T cos_angle = std::cos(angle);
            T sin_angle = std::sin(angle);
            return {_x * cos_angle + _y * sin_angle,
                    -_x * sin_angle + _y * cos_angle};
        }

        void rotate_vector(const T angle)
        {
            T cos_angle = std::cos(angle);
            T sin_angle = std::sin(angle);
            T x_old = _x;
            _x = x_old * cos_angle + _y * sin_angle;
            _y = -x_old * sin_angle + _y * cos_angle;
        }

        // Operator overloading
        Vector2D<T> operator+(const Vector2D<T>& other) const
        {
            return {_x + other._x, 
                    _y + other._y};
        }

        Vector2D<T> operator+(const T other) const
        {
            return {_x + other, 
                    _y + other};
        }

        Vector2D<T> operator-(const Vector2D<T>& other) const
        {
            return {_x - other._x,
                    _y - other._y};
        }

        Vector2D<T> operator-(const T other) const
        {
            return {_x - other,
                    _y - other};
        }

        Vector2D<T> operator*(const Vector2D<T>& other) const
        {
            return {_x * other._x,
                    _y * other._y};
        }

        Vector2D<T> operator*(const T other) const
        {
            return {_x * other,
                    _y * other};
        }

        Vector2D<T> operator/(const Vector2D<T>& other) const
        {
            return {_x / other._x,
                    _y / other._y};
        }

        Vector2D<T> operator/(const T other) const
        {
            return {_x / other,
                    _y / other};
        }

    };

    template <typename T>
    struct Vector3D
    {
    
        // Struct members
        T _x;           // X coordinate
        T _y;           // Y coordinate
        T _heading;     // Heading direction w.r.t. x-axis
 
        // Constructors
        Vector3D() : _x(static_cast<T>(0)), _y(static_cast<T>(0)), _heading(static_cast<T>(0)) {}
        Vector3D(T x, T y, T heading) : _x(x), _y(y), _heading(heading) {}

        // Functions
        Vector3D<T> get_rotated_vector(const T angle) const
        {
            T cos_angle = std::cos(angle);
            T sin_angle = std::sin(angle);
            return {_x * cos_angle + _y * sin_angle,
                    -_x * sin_angle + _y * cos_angle,
                    wrap_pi<T>(_heading - angle)};
        }

        // Operator overloading
        Vector3D<T> operator+(const Vector3D<T>& other) const
        {
            return {_x + other._x, 
                    _y + other._y,
                    _heading + other._heading};
        }

        Vector3D<T> operator-(const Vector3D<T>& other) const
        {
            return {_x - other._x,
                    _y - other._y,
                    _heading - other._heading};
        }

        Vector3D<T> operator*(const Vector3D<T>& other) const
        {
            return {_x * other._x,
                    _y * other._y,
                    _heading * other._heading};
        }

        Vector3D<T> operator/(const Vector3D<T>& other) const
        {
            return {_x / other._x,
                    _y / other._y,
                    _heading / other._heading};
        }

        Vector3D& operator=(const Vector3D<T>& other) 
        {
            if (this != &other) 
            {
                _x = other._x;
                _y = other._y;
                _heading = other._heading;
            }
            
            return *this;
        }

        template <typename U>
        Vector3D& operator=(const Vector3D<U>& other) 
        {
            _x = static_cast<T>(other._x);
            _y = static_cast<T>(other._y);
            _heading = static_cast<T>(other._heading);
        
            return *this;
        }
    };   
}

#endif