#ifndef GRID3D
#define GRID3D

#include <vector>
#include <utility>
#include <numeric>
#include "Grid2D.h"
#include "VehicleModel.h"
#include "common.h"

namespace planning
{
    template <typename T>
    class Grid3D : public Grid2D<T>
    {
    
    public:
        // Constructors
        Grid3D(T resolution, T obstacle_threshold, T obstacle_prob_min, T obstacle_prob_max, 
                T obstacle_prob_free, int grid_size, bool allow_diag_moves, T step_size, 
                T max_lat_acc, T max_long_dec, T wheelbase, T rear_to_cg, T apf_rep_constant, 
                T apf_active_angle, int num_angle_bins, int num_actions, const std::vector<T>& steering, 
                const std::vector<T>& curvature_weights);

        using Grid2D<T>::update_obstacles;
        
        // Public member functions
        void update_obstacles(const std::vector<Obstacle<T>>& obstacles, const std::vector<T>& confidence, 
                const T apf_added_radius);
        bool get_neighbors(const Node3D<T>& node, std::vector<Node3D<T>>& neighbors) const;
        bool check_path(const std::vector<Vector3D<T>>& path) const;
        Vector3D<T> get_goal_location() const;
        Node3D<T> update_goal_heading(const Vector3D<T>& goal, const Vector3D<T>& start);
        Node3D<T> set_start_node(const Vector3D<T>& start);
        const std::vector<T>& get_abs_curvatures() const;

    private:
        // Private member functions
        T get_field_intensity(const Node3D<T>& node) const;

        // Private class members
        const T _apf_rep_constant;                              // Constant coefficient used in calculating repulsive potentials for APF
        const T _apf_active_angle;                              // Angle range within which APF are calculated for obstacles relative to pose heading
        Vector3D<T> _goal_location3D;                           // Stores the current goal's 2D pose for transforming points
        VehicleModel<T> _model;                                 // Vehicle model used to simulate actions and get neighboring states
        std::vector<std::pair<Vector2D<T>, T>> _apf_obstacles;  // Stores the locations and radii of obstacles used for APF   
    };
}

#endif