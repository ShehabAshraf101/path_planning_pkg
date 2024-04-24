#ifndef NODE3D
#define NODE3D

#include <vector>
#include <algorithm>
#include <functional>
#include <boost/functional/hash.hpp>
#include "Node2D.h"
#include "common.h"

namespace planning
{
    template <typename T>
    struct Node3D
    {
        // Struct members
        Vector3D<T> _pose2D;            // 2D pose of the node
        T _cost_g;                      // Accumulated cost of reaching this node from the start
        T _cost_f;                      // Total cost of reaching the goal
        T _vmin_sqr;                    // The minimum velocity squared with which you can arrive at this node (for dynamic constraints)
        int _curvature_index;           // Index of the curvature of the action taken to reach this node
        int _angle_bin;                 // Index of discrete orientation according to set precision (used for equality comparisons)
        const Node2D<T>* _base_node;    // Pointer to the 2D node corresponding to this location in the grid
        const Node3D<T>* _prev;         // Pointer to the predecessor node (if it exists)

        // Constructors
        Node3D(Vector3D<T>& pose2D, T cost_g, T vmin_sqr, int curvature_index, int angle_bin, 
                const Node2D<T>* base_node, const Node3D<T>* prev);
        Node3D(Vector3D<T>& pose2D, T cost_g, T vmin_sqr, int curvature_index, int angle_bin, 
                const Node3D<T>* prev);
        Node3D();

        // Functions
        void set_accumulated_cost(const T cost_g);
        void set_heuristic_cost(const T cost_h);
        void soft_reset(); 

        // Operator overloading
        friend bool operator==(const Node3D<T>& node_left, const Node3D<T>& node_right)
        {
            // heading compared based on angle bins
            return ((*(node_left._base_node) == *(node_right._base_node)) && (node_left._angle_bin == node_left._angle_bin));
        }

        friend bool operator!=(const Node3D<T>& node_left, const Node3D<T>& node_right)
        {
            return ((*(node_left._base_node) != *(node_right._base_node)) || (node_left._angle_bin != node_right._angle_bin));
        }

        friend bool operator<(const Node3D<T>& node_left, const Node3D<T>& node_right)
        {
            // return false if they are equal regardless of cost (for usage in a std::set)
            return (node_left != node_right) && (node_left._cost_f < node_right._cost_f);
        }

        friend bool operator<=(const Node3D<T>& node_left, const Node3D<T>& node_right)
        {
            // same logic as operator< for consistency
            return (node_left != node_right) && (node_left._cost_f <= node_right._cost_f);
        }

        friend bool operator>(const Node3D<T>& node_left, const Node3D<T>& node_right)
        {
            // same logic as operator< for consistency
            return (node_left != node_right) && (node_left._cost_f > node_right._cost_f);
        }

        friend bool operator>=(const Node3D<T>& node_left, const Node3D<T>& node_right)
        {
            // same logic as operator< for consistency
            return (node_left != node_right) && (node_left._cost_f >= node_right._cost_f);
        }

        friend std::ostream& operator<<(std::ostream& os, const Node3D<T>& node)
        {
            if (node._base_node != nullptr)
            {
                os << "xd = " << node._base_node->_posd._x << " yd = " << node._base_node->_posd._y << "\n";
            }

            os << "x = " << node._pose2D._x << " y = " << node._pose2D._y << " heading = " << node._pose2D._heading 
                << "\n" << "cost_g = " << node._cost_g << " cost_h = " << (node._cost_f - node._cost_g)
                << " cost_f = " << node._cost_f << "\n" << "vmin_sqr = " << node._vmin_sqr << " curvature_index = " 
                << node._curvature_index << " angle_bin = " << node._angle_bin << "\n" << std::endl;
            return os; 
        }

        // Define hash function for a Node3D
        struct HashFunction
        {
            size_t operator()(const Node3D<T>& node) const
            {
                std::size_t seed = 0;
                boost::hash_combine(seed, node._base_node->_posd._x);
                boost::hash_combine(seed, node._base_node->_posd._y);
                boost::hash_combine(seed, node._angle_bin);
                return std::hash<size_t>()(seed);
            }
        };
    };
}

#endif