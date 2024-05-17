#include "Node2D.h"

using namespace planning;

// Constructors
template <typename T>
Node2D<T>::Node2D(int xd, int yd, T cost_g, T cost_h, const Node2D<T>* prev) :
        _posd(xd, yd), 
        _cost_g(cost_g), 
        _cost_h(cost_h),
        _cost_f(cost_g + cost_h), 
        _prev(prev) {}

template <typename T>
Node2D<T>::Node2D(int xd, int yd) : 
        Node2D(xd, yd, static_cast<T>(0), static_cast<T>(0), nullptr) {}


// Public functions
template <typename T>
void Node2D<T>::set_accumulated_cost(const T cost_g)
{
    _cost_g = cost_g;
    _cost_f = cost_g + _cost_h;
}

template <typename T>
void Node2D<T>::set_heuristic_cost(const T cost_h)
{
    _cost_h = cost_h;
    _cost_f = _cost_g + cost_h;
}

template <typename T>
void Node2D<T>::soft_reset() // Maintains xd, yd and cost_h
{
    _cost_g = static_cast<T>(0);
    _cost_f = _cost_h;
    _prev   = nullptr;
}

// Explicit instantiation of supported types
template class Node2D<float>;
template class Node2D<double>;