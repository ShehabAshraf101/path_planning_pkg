#include "Dubins.h"

using namespace planning;

// Contructors
template <typename T>
Dubins<T>::Dubins(T r_min, T step_size) : 
        _r_min(r_min), 
        _step_size(step_size), 
        _ang_step_size(step_size/r_min), 
        _path_type(Path::RSR) 
{
    _params.fill(static_cast<T>(0));
}


// Public functions
template <typename T>
T Dubins<T>::get_shortest_path_length(const Vector3D<T>& start, const Vector3D<T>& goal)
{
    // determine the centers of both right and left circles
    Vector2D<T> center_s_r, center_s_l, center_g_r, center_g_l;
    center_s_r._x = start._x + _r_min * std::sin(start._heading);
    center_s_r._y = start._y - _r_min * std::cos(start._heading);

    center_s_l._x = start._x - _r_min * std::sin(start._heading);
    center_s_l._y = start._y + _r_min * std::cos(start._heading);
    
    center_g_r._x = goal._x + _r_min * std::sin(goal._heading);
    center_g_r._y = goal._y - _r_min * std::cos(goal._heading);
    
    center_g_l._x = goal._x - _r_min * std::sin(goal._heading);
    center_g_l._y = goal._y + _r_min * std::cos(goal._heading);
    
    // start with RSR path
    std::array<T, 4> path_params;
    T path_length_min = get_params_rsr(center_s_r, center_g_r, start, goal, path_params);
    _params = path_params;
    _path_type = Path::RSR;

    // RSL path
    T path_length = get_params_rsl(center_s_r, center_g_l, start, goal, path_params);
    if (path_length < path_length_min)
    {
        _params = path_params;
        _path_type = Path::RSL;
        path_length_min = path_length;
    }

    // LSR path
    path_length = get_params_lsr(center_s_l, center_g_r, start, goal, path_params);
    if (path_length < path_length_min)
    {
        _params = path_params;
        _path_type = Path::LSR;
        path_length_min = path_length;
    }

    // LSL path
    path_length = get_params_lsl(center_s_l, center_g_l, start, goal, path_params);
    if (path_length < path_length_min)
    {
        _params = path_params;
        _path_type = Path::LSL;
        path_length_min = path_length;
    }

    return path_length_min;
}

template <typename T>
T Dubins<T>::get_shortest_path_length(const Vector3D<T>& start, const Vector3D<T>& goal, Vector2D<T>& center_s_r, 
                Vector2D<T>& center_s_l, Vector2D<T>& center_g_r, Vector2D<T>& center_g_l)
{
    // determine the centers of both right and left circles
    center_s_r._x = start._x + _r_min * std::sin(start._heading);
    center_s_r._y = start._y - _r_min * std::cos(start._heading);

    center_s_l._x = start._x - _r_min * std::sin(start._heading);
    center_s_l._y = start._y + _r_min * std::cos(start._heading);
    
    center_g_r._x = goal._x + _r_min * std::sin(goal._heading);
    center_g_r._y = goal._y - _r_min * std::cos(goal._heading);
    
    center_g_l._x = goal._x - _r_min * std::sin(goal._heading);
    center_g_l._y = goal._y + _r_min * std::cos(goal._heading);
    
    // start with RSR path
    std::array<T, 4> path_params;
    T path_length_min = get_params_rsr(center_s_r, center_g_r, start, goal, path_params);
    _params = path_params;
    _path_type = Path::RSR;

    // RSL path
    T path_length = get_params_rsl(center_s_r, center_g_l, start, goal, path_params);
    if (path_length < path_length_min)
    {
        _params = path_params;
        _path_type = Path::RSL;
        path_length_min = path_length;
    }

    // LSR path
    path_length = get_params_lsr(center_s_l, center_g_r, start, goal, path_params);
    if (path_length < path_length_min)
    {
        _params = path_params;
        _path_type = Path::LSR;
        path_length_min = path_length;
    }

    // LSL path
    path_length = get_params_lsl(center_s_l, center_g_l, start, goal, path_params);
    if (path_length < path_length_min)
    {
        _params = path_params;
        _path_type = Path::LSL;
        path_length_min = path_length;
    }

    return path_length_min;
}

template <typename T>
std::pair<T, bool> Dubins<T>::get_shortest_path(const Vector3D<T>& start, const Vector3D<T>& goal, 
                            std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature)
{
    // determine shortest path and store center locations
    Vector2D<T> center_s_r, center_s_l, center_g_r, center_g_l;
    T path_length_min = get_shortest_path_length(start, goal, center_s_r, center_s_l, center_g_r, center_g_l);

    // sample the shortest path based on its type
    switch (_path_type)
    {
    case Path::RSR:
        sample_path_rsr(center_s_r, center_g_r, path, path_curvature);
        break;
    
    case Path::RSL:
        sample_path_rsl(center_s_r, center_g_l, path, path_curvature);
        break;

    case Path::LSR:
        sample_path_lsr(center_s_l, center_g_r, path, path_curvature);
        break;

    case Path::LSL:
        sample_path_lsl(center_s_l, center_g_l, path, path_curvature);
        break;
    }

    return std::pair<T, bool>(path_length_min, std::abs(_params[1]) > static_cast<T>(M_PI_2));
}

template <typename T>
std::string Dubins<T>::get_path_type() const
{
    switch (_path_type)
    {
        case Path::RSR: 
            return std::string("RSR");
        
        case Path::RSL: 
            return std::string("RSL");

        case Path::LSR: 
            return std::string("LSR");

        case Path::LSL:
            return std::string("LSL");
    }

    return std::string("undefined"); 
}



// Private functions
template <typename T>
T Dubins<T>::get_params_rsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
        const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const
{
    // offset between both circles' centers
    Vector2D<T> delta_center = center_g - center_s;
    T theta = std::atan2(delta_center._y, delta_center._x);

    // calculating the 4 angles (start, tangent 1, tangent 2, goal)
    params[0]  = M_PI_2 + start._heading;
    T theta_t1 = M_PI_2 + theta;
    params[2]  = theta_t1;
    T theta_g  = M_PI_2 + goal._heading;

    // delta angles for each circle
    params[1] = theta_t1 - params[0];
    if (params[1] > 0)
    {
        params[1] -= 2 * M_PI;
    }
    params[3] = theta_g - params[2];
    if (params[3] > 0)
    {
        params[3] -= 2 * M_PI;
    }

    // calculate the straight path length
    T dist_st = std::sqrt(delta_center._x * delta_center._x + delta_center._y * delta_center._y);

    return dist_st + _r_min * -(params[1] + params[3]);
}

template <typename T>
T Dubins<T>::get_params_rsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
        const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const
{
    // offset between both circles' centers
    Vector2D<T> delta_center = center_g - center_s;
    T dist  = std::sqrt(delta_center._x * delta_center._x + delta_center._y * delta_center._y);
    T theta = std::atan2(delta_center._y, delta_center._x);

    // calculating the 4 angles (start, tangent 1, tangent 2, goal)
    params[0]  = M_PI_2 + start._heading;
    T theta_t1 = std::acos(2 * _r_min/dist) + theta;
    params[2]  = theta_t1 - M_PI;
    T theta_g  = -M_PI_2 + goal._heading;

    // delta angles for each circle
    params[1] = theta_t1 - params[0];
    if (params[1] > 0)
    {
        params[1] -= 2 * M_PI;
    }
    params[3] = theta_g - params[2];
    if (params[3] < 0)
    {
        params[3] += 2 * M_PI;
    }

    // calculate the straight path length
    Vector2D<T> start_st = center_s, end_st = center_g;
    start_st._x += _r_min * std::cos(theta_t1);
    start_st._y += _r_min * std::sin(theta_t1);
    
    end_st._x += _r_min * std::cos(params[2]);
    end_st._y += _r_min * std::sin(params[2]);
    
    Vector2D<T> delta_st = end_st - start_st;
    T dist_st = std::sqrt(delta_st._x * delta_st._x + delta_st._y * delta_st._y);

    return dist_st + _r_min * (-params[1] + params[3]);
}

template <typename T>
T Dubins<T>::get_params_lsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
        const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const
{
    // offset between both circles' centers
    Vector2D<T> delta_center = center_g - center_s;
    T dist  = std::sqrt(delta_center._x * delta_center._x + delta_center._y * delta_center._y);
    T theta = std::atan2(delta_center._y, delta_center._x);

    // calculating the 4 angles (start, tangent 1, tangent 2, goal)
    params[0]  = -M_PI_2 + start._heading;
    T theta_t1 = -std::acos(2 * _r_min/dist) + theta;
    params[2]  = theta_t1 + M_PI;
    T theta_g  = M_PI_2 + goal._heading;

    // delta angles for each circle
    params[1] = theta_t1 - params[0];
    if (params[1] < 0)
    {
        params[1] += 2 * M_PI;
    }
    params[3] = theta_g - params[2];
    if (params[3] > 0)
    {
        params[3] -= 2 * M_PI;
    }

    // calculate the straight path length
    Vector2D<T> start_st = center_s, end_st = center_g;
    start_st._x += _r_min * std::cos(theta_t1);
    start_st._y += _r_min * std::sin(theta_t1);

    end_st._x += _r_min * std::cos(params[2]);
    end_st._y += _r_min * std::sin(params[2]);
    
    Vector2D<T> delta_st = end_st - start_st;
    T dist_st = std::sqrt(delta_st._x * delta_st._x + delta_st._y * delta_st._y);

    return dist_st + _r_min * (params[1] - params[3]);   
}

template <typename T>
T Dubins<T>::get_params_lsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
        const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const
{
    // offset between both circles' centers
    Vector2D<T> delta_center = center_g - center_s;
    T theta = std::atan2(delta_center._y, delta_center._x);

    // calculating the 4 angles (start, tangent 1, tangent 2, goal)
    params[0]  = -M_PI_2 + start._heading;
    T theta_t1 = -M_PI_2 + theta;
    params[2]  = theta_t1;
    T theta_g  = -M_PI_2 + goal._heading;

    // delta angles for each circle
    params[1] = theta_t1 - params[0];
    if (params[1] < 0)
    {
        params[1] += 2 * M_PI;
    }
    params[3] = theta_g - params[2];
    if (params[3] < 0)
    {
        params[3] += 2 * M_PI;
    }

    // calculate the straight path length
    T dist_st = std::sqrt(delta_center._x * delta_center._x + delta_center._y * delta_center._y);

    return dist_st + _r_min * (params[1] + params[3]);
}

template <typename T>
void Dubins<T>::sample_path_rsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const
{
    // determine starting and end points of straight segment
    Vector2D<T> start_st = center_s, end_st = center_g;
    start_st._x += _r_min * std::cos(_params[0] + _params[1]);
    start_st._y += _r_min * std::sin(_params[0] + _params[1]);

    end_st._x += _r_min * std::cos(_params[2]);
    end_st._y += _r_min * std::sin(_params[2]);

    // calculate the total path lengths
    Vector2D<T> delta_st = end_st - start_st;
    T length_st = std::sqrt(delta_st._x * delta_st._x + delta_st._y * delta_st._y);
    int size_1 = static_cast<int>(std::floor(-_params[1]/_ang_step_size));
    int size_2 = size_1 + static_cast<int>(std::floor(length_st/_step_size));
    int size_3 = size_2 + static_cast<int>(std::floor(-_params[3]/_ang_step_size));
    path.resize(size_3 + 1);
    path_curvature.resize(size_3 + 1);

    // sample each of the three segments in succession
    T theta = _params[0], curvature = 1/_r_min;
    for (int i = 0; i < size_1 ; i++)
    {
        path[i]._x = center_s._x + _r_min * std::cos(theta);
        path[i]._y = center_s._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta - M_PI_2);
        path_curvature[i] = curvature;
        theta -= _ang_step_size;
    }

    theta = std::atan2(delta_st._y, delta_st._x);
    T cos_theta = std::cos(theta), sin_theta = std::sin(theta), dist = static_cast<T>(0);
    for (int i = size_1; i < size_2; i++)
    {
        path[i]._x = start_st._x + dist * cos_theta;
        path[i]._y = start_st._y + dist * sin_theta;
        path[i]._heading = theta;
        path_curvature[i] = static_cast<T>(0);
        dist += _step_size;
    }

    theta = _params[2];
    for (int i = size_2; i < size_3; i++)
    {
        path[i]._x = center_g._x + _r_min * std::cos(theta);
        path[i]._y = center_g._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta - M_PI_2);
        path_curvature[i] = curvature;
        theta -= _ang_step_size;
    }

    // add final goal point
    path[size_3]._x = center_g._x + _r_min * std::cos(_params[2] + _params[3]);
    path[size_3]._y = center_g._y + _r_min * std::sin(_params[2] + _params[3]);
    path[size_3]._heading = wrap_pi(_params[2] + _params[3] - M_PI_2);
    path_curvature[size_3] = static_cast<T>(0); 
}

template <typename T>
void Dubins<T>::sample_path_rsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const
{
    // determine starting and end points of straight segment
    Vector2D<T> start_st = center_s, end_st = center_g;
    start_st._x += _r_min * std::cos(_params[0] + _params[1]);
    start_st._y += _r_min * std::sin(_params[0] + _params[1]);

    end_st._x += _r_min * std::cos(_params[2]);
    end_st._y += _r_min * std::sin(_params[2]);

    // calculate the total path lengths
    Vector2D<T> delta_st = end_st - start_st;
    T length_st = std::sqrt(delta_st._x * delta_st._x + delta_st._y * delta_st._y);
    int size_1 = static_cast<int>(std::floor(-_params[1]/_ang_step_size));
    int size_2 = size_1 + static_cast<int>(std::floor(length_st/_step_size));
    int size_3 = size_2 + static_cast<int>(std::floor(_params[3]/_ang_step_size));
    path.resize(size_3 + 1);
    path_curvature.resize(size_3 + 1);

    // sample each of the three segments in succession
    T theta = _params[0], curvature = 1/_r_min;
    for (int i = 0; i < size_1 ; i++)
    {
        path[i]._x = center_s._x + _r_min * std::cos(theta);
        path[i]._y = center_s._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta - M_PI_2);
        path_curvature[i] = curvature;
        theta -= _ang_step_size;
    }

    theta = std::atan2(delta_st._y, delta_st._x);
    T cos_theta = std::cos(theta), sin_theta = std::sin(theta), dist = static_cast<T>(0);
    for (int i = size_1; i < size_2; i++)
    {
        path[i]._x = start_st._x + dist * cos_theta;
        path[i]._y = start_st._y + dist * sin_theta;
        path[i]._heading = theta;
        path_curvature[i] = static_cast<T>(0);
        dist += _step_size;
    }

    theta = _params[2];
    for (int i = size_2; i < size_3; i++)
    {
        path[i]._x = center_g._x + _r_min * std::cos(theta);
        path[i]._y = center_g._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta + M_PI_2);
        path_curvature[i] = curvature;
        theta += _ang_step_size;
    }

    // add final goal point
    path[size_3]._x = center_g._x + _r_min * std::cos(_params[2] + _params[3]);
    path[size_3]._y = center_g._y + _r_min * std::sin(_params[2] + _params[3]);
    path[size_3]._heading = wrap_pi(_params[2] + _params[3] + M_PI_2);
    path_curvature[size_3] = static_cast<T>(0); 
} 

template <typename T>
void Dubins<T>::sample_path_lsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g,
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const
{
    // determine starting and end points of straight segment
    Vector2D<T> start_st = center_s, end_st = center_g;
    start_st._x += _r_min * std::cos(_params[0] + _params[1]);
    start_st._y += _r_min * std::sin(_params[0] + _params[1]);

    end_st._x += _r_min * std::cos(_params[2]);
    end_st._y += _r_min * std::sin(_params[2]);

    // calculate the total path lengths
    Vector2D<T> delta_st = end_st - start_st;
    T length_st = std::sqrt(delta_st._x * delta_st._x + delta_st._y * delta_st._y);
    int size_1 = static_cast<int>(std::floor(_params[1]/_ang_step_size));
    int size_2 = size_1 + static_cast<int>(std::floor(length_st/_step_size));
    int size_3 = size_2 + static_cast<int>(std::floor(-_params[3]/_ang_step_size));
    path.resize(size_3 + 1);
    path_curvature.resize(size_3 + 1);

    // sample each of the three segments in succession
    T theta = _params[0], curvature = 1/_r_min;
    for (int i = 0; i < size_1 ; i++)
    {
        path[i]._x = center_s._x + _r_min * std::cos(theta);
        path[i]._y = center_s._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta + M_PI_2);
        path_curvature[i] = curvature;
        theta += _ang_step_size;
    }

    theta = std::atan2(delta_st._y, delta_st._x);
    T cos_theta = std::cos(theta), sin_theta = std::sin(theta), dist = static_cast<T>(0);
    for (int i = size_1; i < size_2; i++)
    {
        path[i]._x = start_st._x + dist * cos_theta;
        path[i]._y = start_st._y + dist * sin_theta;
        path[i]._heading = theta;
        path_curvature[i] = static_cast<T>(0);
        dist += _step_size;
    }

    theta = _params[2];
    for (int i = size_2; i < size_3; i++)
    {
        path[i]._x = center_g._x + _r_min * std::cos(theta);
        path[i]._y = center_g._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta - M_PI_2);
        path_curvature[i] = curvature;
        theta -= _ang_step_size;
    }

    // add final goal point
    path[size_3]._x = center_g._x + _r_min * std::cos(_params[2] + _params[3]);
    path[size_3]._y = center_g._y + _r_min * std::sin(_params[2] + _params[3]);
    path[size_3]._heading = wrap_pi(_params[2] + _params[3] - M_PI_2);
    path_curvature[size_3] = static_cast<T>(0); 
} 

template <typename T>
void Dubins<T>::sample_path_lsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const
{
    // determine starting and end points of straight segment
    Vector2D<T> start_st = center_s, end_st = center_g;
    start_st._x += _r_min * std::cos(_params[0] + _params[1]);
    start_st._y += _r_min * std::sin(_params[0] + _params[1]);

    end_st._x += _r_min * std::cos(_params[2]);
    end_st._y += _r_min * std::sin(_params[2]);

    // calculate the total path lengths
    Vector2D<T> delta_st = end_st - start_st;
    T length_st = std::sqrt(delta_st._x * delta_st._x + delta_st._y * delta_st._y);
    int size_1 = static_cast<int>(std::floor(_params[1]/_ang_step_size));
    int size_2 = size_1 + static_cast<int>(std::floor(length_st/_step_size));
    int size_3 = size_2 + static_cast<int>(std::floor(_params[3]/_ang_step_size));
    path.resize(size_3 + 1);
    path_curvature.resize(size_3 + 1);

    // sample each of the three segments in succession
    T theta = _params[0], curvature = 1/_r_min;
    for (int i = 0; i < size_1 ; i++)
    {
        path[i]._x = center_s._x + _r_min * std::cos(theta);
        path[i]._y = center_s._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta + M_PI_2);
        path_curvature[i] = curvature;
        theta += _ang_step_size;
    }

    theta = std::atan2(delta_st._y, delta_st._x);
    T cos_theta = std::cos(theta), sin_theta = std::sin(theta), dist = static_cast<T>(0);
    for (int i = size_1; i < size_2; i++)
    {
        path[i]._x = start_st._x + dist * cos_theta;
        path[i]._y = start_st._y + dist * sin_theta;
        path[i]._heading = theta;
        path_curvature[i] = static_cast<T>(0);
        dist += _step_size;
    }

    theta = _params[2];
    for (int i = size_2; i < size_3; i++)
    {
        path[i]._x = center_g._x + _r_min * std::cos(theta);
        path[i]._y = center_g._y + _r_min * std::sin(theta);
        path[i]._heading = wrap_pi(theta + M_PI_2);
        path_curvature[i] = curvature;
        theta += _ang_step_size;
    }

    // add final goal point
    path[size_3]._x = center_g._x + _r_min * std::cos(_params[2] + _params[3]);
    path[size_3]._y = center_g._y + _r_min * std::sin(_params[2] + _params[3]);
    path[size_3]._heading = wrap_pi(_params[2] + _params[3] + M_PI_2);
    path_curvature[size_3] = static_cast<T>(0); 
}

// Explicit instantiation of supported types
template class Dubins<float>;
template class Dubins<double>;