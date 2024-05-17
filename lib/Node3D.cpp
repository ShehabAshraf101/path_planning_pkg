#include "Node3D.h"

using namespace planning;

// Constructors
template <typename T>
Node3D<T>::Node3D(Vector3D<T>& pose2D, T cost_g, T vmin_sqr, int curvature_index, int angle_bin, 
                const Node2D<T>* base_node, const Node3D<T>* prev) :
        _pose2D(pose2D), 
        _cost_g(cost_g), 
        _cost_f(cost_g), 
        _vmin_sqr(vmin_sqr), 
        _curvature_index(curvature_index), 
        _angle_bin(angle_bin),
        _base_node(base_node), 
        _prev(prev) {}

template <typename T>
Node3D<T>::Node3D(Vector3D<T>& pose2D, T cost_g, T vmin_sqr, int curvature_index, int angle_bin, 
                const Node3D<T>* prev) : 
        Node3D(pose2D, cost_g, vmin_sqr, curvature_index, angle_bin, nullptr, prev) {}

template <typename T>
Node3D<T>::Node3D() : 
        _pose2D(Vector3D<T>()), 
        _cost_g(static_cast<T>(0)), 
        _cost_f(static_cast<T>(0)), 
        _vmin_sqr(static_cast<T>(0)), 
        _curvature_index(0), 
        _angle_bin(0),
        _base_node(nullptr), 
        _prev(nullptr) {}

// Functions
template <typename T>
void Node3D<T>::set_accumulated_cost(const T cost_g)
{
    _cost_g = cost_g;
    _cost_f += cost_g;
}

template <typename T>
void Node3D<T>::set_heuristic_cost(const T cost_h)
{
    if (_base_node != nullptr)
    {
        _cost_f += std::max(cost_h, _base_node->_cost_f);
    }
    else
    {
        _cost_f += cost_h;
    }
}

template <typename T>
void Node3D<T>::soft_reset()
{
    _cost_g = static_cast<T>(0);
    _cost_f = static_cast<T>(0);
    // _base_node->soft_reset();
    _prev = nullptr;
}

// Explicit instantiation of supported types
template class Node3D<float>;
template class Node3D<double>;