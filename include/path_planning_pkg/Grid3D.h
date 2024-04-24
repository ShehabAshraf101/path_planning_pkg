#ifndef GRID3D
#define GRID3D

#include <vector>
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
        Grid3D(T resolution, int grid_size, bool allow_diag_moves, T step_size, T max_lat_acc, T max_long_dec,
                T wheelbase, T rear_to_cg, int num_angle_bins, int num_actions, const std::vector<T>& steering, 
                const std::vector<T>& curvature_weights);

        // Public member functions
        bool get_neighbors(const Node3D<T>& node, std::vector<Node3D<T>>& neighbors) const;
        bool check_path(const std::vector<Vector3D<T>>& path) const;
        Node3D<T> update_goal_heading(const Vector3D<T>& goal, const Vector3D<T>& start);
        Node3D<T> set_start_node(const Vector3D<T>& start);
        const std::vector<T>& get_abs_curvatures() const;

    private:
        // Private class members
        Vector3D<T> _goal_location3D;
        VehicleModel<T> _model;
    };
}

#endif