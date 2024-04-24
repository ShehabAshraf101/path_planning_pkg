#ifndef NODE2D
#define NODE2D

#include <iostream>
#include <functional>
#include <boost/functional/hash.hpp>
#include "common.h"

namespace planning
{
    template <typename T> 
    struct Node2D
    {
        // Struct members
        Vector2D<int> _posd;        // Discrete position in xy plane (indices in a grid)
        T _cost_g;                  // The accumulated cost to reach this node from the start
        T _cost_h;                  // The estimated cost to reach the goal from this node by the heuristic
        T _cost_f;                  // The total cost of reaching the goal (F = G + H)
        const Node2D<T>* _prev;     // Pointer to the predecessor node (if it exists)

        // Constructors
        Node2D(int xd, int yd, T cost_g, T cost_h, const Node2D<T>* prev);
        Node2D(int xd, int yd);

        // Functions
        void set_accumulated_cost(const T cost_g);
        void set_heuristic_cost(const T cost_h);
        void soft_reset(); // Maintains xd, yd and cost_h

        // Operator overloading
        friend bool operator==(const Node2D<T>& node_left, const Node2D<T>& node_right)
        {
            return ((node_left._posd._x == node_right._posd._x) && (node_left._posd._y == node_right._posd._y));
        }

        friend bool operator!=(const Node2D<T>& node_left, const Node2D<T>& node_right)
        {
            return ((node_left._posd._x != node_right._posd._x) || (node_left._posd._y != node_right._posd._y));
        }

        friend bool operator<(const Node2D<T>& node_left, const Node2D<T>& node_right)
        {
            // return false if they are equal regardless of cost (for usage in a std::set)
            return (node_left != node_right) && (node_left._cost_f < node_right._cost_f);
        }

        friend bool operator<=(const Node2D<T>& node_left, const Node2D<T>& node_right)
        {
            // same logic as operator< for consistency
            return (node_left != node_right) && (node_left._cost_f <= node_right._cost_f);
        }

        friend bool operator>(const Node2D<T>& node_left, const Node2D<T>& node_right)
        {
            // same logic as operator< for consistency
            return (node_left != node_right) && (node_left._cost_f > node_right._cost_f);
        }

        friend bool operator>=(const Node2D<T>& node_left, const Node2D<T>& node_right)
        {
            // same logic as operator< for consistency
            return (node_left != node_right) && (node_left._cost_f >= node_right._cost_f);
        }

        friend std::ostream& operator<<(std::ostream& os, const Node2D<T>& node)
        {
            os << "xd = " << node._posd._x << " yd = " << node._posd._y << "\n"
                << "cost_g = " << node._cost_g << " cost_h = " << node._cost_h 
                << " cost_f = " << node._cost_f << "\n" << std::endl;
            return os; 
        }

        // Define hash function for a Node2D
        struct HashFunction
        {
            size_t operator()(const Node2D<T>& node) const
            {
                std::size_t seed = 0;
                boost::hash_combine(seed, node._posd._x);
                boost::hash_combine(seed, node._posd._y);
                
                return std::hash<size_t>()(seed);
            }
        };
    };
}

#endif