#ifndef VEHICLE_MODEL
#define VEHICLE_MODEL

#include <cmath>
#include <vector>
#include <utility>
#include "Node3D.h"
#include "common.h"

namespace planning
{
    template <typename T>
    class VehicleModel
    {
    
    public:
        // Constructors
        VehicleModel(T ts, T max_lat_acc, T max_long_dec, T wheelbase, T rear_to_cg, 
                int num_angle_bins, int num_actions, const std::vector<T>& steering, 
                const std::vector<T>& curvature_weights);

        // Public member functions
        T get_precision() const;
        int get_default_action_index() const;
        bool get_neighbors(const Node3D<T>& node, std::vector<Node3D<T>>& neighbors) const;
        std::pair<bool, Node3D<T>> simulate_action(const Node3D<T>& node, const int action_index) const;
        const std::vector<T>& get_abs_curvatures() const;

    private:
        // Private member functions
        Vector2D<T> calculate_offset(const T beta, const T curvature, const T heading) const;

        // Private member variables
        const T _ts;                                        // Sampling time for prediction (also equals distance traveled if V = 1m/s)
        const T _max_lat_acc;                               // Maximum allowable lateral acceleration (m/s^2)
        const T _max_lat_acc_sqr;                           // Maximum allowable lateral acceleration squared
        const T _max_long_dec;                              // Maximum allowable longitudinal acceleration during braking (m/s^2)
        const T _precision;                                 // Precision used for discretizing heading into discrete angle bins (rad)
        const int _num_actions;                             // Odd integer (min = 1) which determines the number of actions explored neighboring
                                                            // the previous action (1 = prev action + one action to its left and right)
        std::vector<T> _abs_curvatures;                     // Stores the absolute curvatures of each possible action
        std::vector<T> _actions_cost;                       // Stores the cost (distance travelled) of taking each action
        std::vector<T> _offset_heading;                     // Stores the offset in heading of taking each action
        std::vector<std::vector<Vector2D<T>>> _offset_xy;   // Stores the offset along x and y (as Vector2D) of taking each action given it and 
                                                            // the discrete heading of the previous state
    };
}

#endif