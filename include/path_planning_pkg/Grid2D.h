#ifndef GRID2D
#define GRID2D

#include <cmath>
#include <vector>
#include <utility>
#include "Node2D.h"
#include "Obstacle.h"
#include "common.h"

namespace planning
{
    template <typename T>
    class Grid2D
    {
    
    public:
        // Constructors
        Grid2D(T resolution, int grid_size, Vector2D<T> goal, Vector2D<T> start, bool allow_diag_moves);
        Grid2D(T resolution, int grid_size, bool allow_diag_moves);

        // Public functions
        void get_neighbors(const int xd, const int yd, std::vector<std::pair<Node2D<T>*, T>>& neighbors);
        void update_obstacles(const std::vector<Obstacle<T>>& obstacles);
        void clear_obstacles();
        void update_costs(const T total_cost, const Node2D<T>& last_node);
        T get_node_total_cost(const int i, const int j) const;
        T get_grid_heading() const;
        T get_grid_resolution() const;
        int get_grid_size() const;
        Node2D<T> update_goal_heading(const Vector2D<T>& goal, const Vector2D<T>& start);
        Node2D<T> set_start_node(const Vector2D<T>& start);
        Node2D<T> set_start_node_grid(const int i, const int j);

    protected:
        // Protected functions
        void compute_heuristic();

        // Protected class members
        T _grid_heading;                                    // Current heading of the 2D grid in 3D space
        const T _resolution;                                // Resolution in meters (the actual size of one cell in the grid)
        const int _grid_size;                               // Number of cells along the axes of the grid (square grid)
        const int _grid_size_2;                             // Grid size / 2
        const int _grid_size_4_5;                           // Grid size * 4/5
        Vector2D<T> _goal_location;                         // Goal 2D coordinates (for offsetting other locations relative to it)
        std::vector<std::vector<Node2D<T>>> _node_map;      // 2D vector for storing nodes
        std::vector<std::vector<bool>> _obstacle_map;       // 2D vector for storing obstacle status (true = obstacle)
        std::vector<std::pair<int, int>> _actions;          // Vector of actions as pair of ints
        std::vector<T> _actions_cost;                       // Vector storing the cost of assoicated actions
    };
}

#endif