#ifndef HYBRID_ASTAR
#define HYBRID_ASTAR

// Changes the way the 2D grid is stored inside AStar<T> in order to have both
// AStar<T> and HybridAStar<T> sharing the same 2D grid
// #define STORE_GRID_AS_REFERENCE

#include <vector>
#include <set>
#include <unordered_set>
#include <algorithm>
#include <utility>
#include "Grid3D.h"
#include "Dubins.h"
#include "AStar.h"
#include "common.h"

namespace planning
{
    // Simple helper function for getting the tan() of the max element in a vector
    template <typename T>
    T tan_max(const std::vector<T>& vect)
    {
        return std::tan(*std::max_element(vect.begin(), vect.end()));
    }

    template <typename T>
    class HybridAStar
    {
    
    public:
        // Contructors
        HybridAStar(int dubins_shot_interval, int dubins_shot_interval_decay, T grid_resolution, 
                T obstacle_threshold, T obstacle_prob_min, T obstacle_prob_max, T obstacle_prob_free, 
                int grid_size, bool grid_2d_allow_diag_moves, T step_size, T max_lat_acc, 
                T max_long_dec, T wheelbase, T rear_to_cg, T apf_rep_constant, T apf_active_angle, 
                int num_angle_bins, int num_actions, const std::vector<T>& steering, 
                const std::vector<T>& curvature_weights);

        // Public member functions
        void update_obstacles(const std::vector<Obstacle<T>>& obstacles, const std::vector<T>& confidence, 
                const T apf_added_radius);
        void update_obstacles(const std::vector<std::pair<Vector2D<T>, Vector2D<T>>>& lines, 
                const std::vector<T>& confidence, const T line_width);
        void update_obstacles();
        void reset();
        void update_goal(const Vector3D<T>& goal, const Vector3D<T>& start);
        const std::vector<std::vector<T>>& get_obstacles() const;
        std::pair<T, bool> find_path(const T vel_init, const Vector3D<T>& start, 
                std::vector<Vector3D<T>>& path, std::vector<T>& curvature);
        
    private:
        // Private member functions
        std::pair<T, bool> hybrid_a_star_search(Node3D<T>& start_node);
        Node3D<T> update_start(const Vector3D<T>& start);
        void reconstruct_path(const Vector3D<T>& goal, const Vector3D<T>& goal_grid, std::vector<Vector3D<T>>& path, 
                std::vector<T>& curvature) const;
        void choose_alternative_goal();

        // Private class members
        int _dubins_shot_interval;                                  // Interval (in terms of nodes expanded) for attempting dubins shots
        int _dubins_shot_interval_decay;                            // Interval is decremented by this number after each dubins shot      
        bool _dubins_shot_successful;                               // Stores the search terminated with a successful dubins shot or not
        std::vector<T> _dubins_abs_curvatures;                      // Stores the absolute curvature the dubins path
        std::vector<Vector3D<T>> _dubins_path;                      // Stores the dubins path  
        Node3D<T> _goal_node;                                       // Stores a copy of the goal node to be reached
        Node3D<T> _terminal_node;                                   // Stores a copy of the last node reached before terminating Hybrid A*
        Grid3D<T> _grid;                                            // 3D grid representing the search space
        AStar<T> _astar;                                            // Calculates shortest holonomic path with obstacles using A*
        Dubins<T> _dubins;                                          // Calculates and generates shortest non-holonomic obstacle 
                                                                    // free path using Dubins paths
        std::set<Node3D<T>> _open_set;                              // Stores candidate nodes to be explored in order (least cost first)
        std::unordered_set<Node3D<T>, 
                typename Node3D<T>::HashFunction> _closed_set;      // Stores nodes that have been fully explored
    };
}

#endif