#ifndef ASTAR
#define ASTAR

#include <vector>
#include <set>
#include <unordered_set>
#include <utility>
#include <limits>
#include <algorithm>
#include "Grid2D.h"
#include "common.h"

namespace planning
{
    template <typename T>
    class AStar
    {
    
    public:
        // Contructors
        #ifndef STORE_GRID_AS_REFERENCE
        AStar(T grid_resolution, T obstacle_threshold, T obstacle_prob_min, T obstacle_prob_max, 
                T obstacle_prob_free, int grid_size, bool grid_allow_diag_moves=true);
        #else
        AStar(Grid2D<T>& grid);
        #endif

        // Public functions
        void update_goal_node(const Node2D<T>& goal_node);
        void update_goal_start(const Vector2D<T>& goal, const Vector2D<T>& start, Node2D<T>& start_node);
        void update_obstacles(const std::vector<Obstacle<T>>& obstacles, const std::vector<T>& confidence);
        void update_obstacles(const std::vector<std::pair<Vector2D<T>, Vector2D<T>>>& lines, 
                const std::vector<T>& confidence, const T line_width);
        void update_obstacles();
        void reset();
        const std::vector<std::vector<T>>& get_obstacles() const;
        T find_path(const Vector2D<T>& goal, const Vector2D<T>& start, std::vector<Vector2D<T>>& path);
        T find_path(const Vector2D<T>& goal, const Vector2D<T>& start, bool get_cost_only=true);
        T find_path(const int start_i, const int start_j);

    private:
        // Private member functions
        T a_star_search(Node2D<T>& start_node, bool get_cost_only=true);
        void reconstruct_path(const Vector2D<T>& goal, std::vector<Vector2D<T>>& path) const;
        void update_visted(const T total_cost, const Node2D<T>& last_node);

        // Private class members
        Node2D<T> _goal_node;                                     // Stores a copy of the goal node to be reached
        #ifndef STORE_GRID_AS_REFERENCE
        Grid2D<T> _grid;                                          // 2D grid representing the search space
        #else
        Grid2D<T>& _grid;                                         // Reference to 2D grid representing the search space
        #endif
        std::vector<std::vector<bool>> _visted;                   // Stores which nodes already have a known path to the goal
        std::set<Node2D<T>> _open_set;                            // Stores candidate nodes to be explored in order (least cost first)
        std::unordered_set<Node2D<T>, 
                typename Node2D<T>::HashFunction> _closed_set;    // Stores nodes that have been fully explored
    };
}

#endif