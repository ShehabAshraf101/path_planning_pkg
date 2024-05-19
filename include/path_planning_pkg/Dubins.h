#ifndef DUBINS
#define DUBINS

#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <utility>
#include "common.h"

namespace planning
{
    enum class Path 
    {
        RSR,
        RSL,
        LSR,
        LSL
    };

    template <typename T>
    class Dubins
    {

    public:
        // Contructors
        Dubins(T r_min, T step_size);

        // Public functions
        T get_shortest_path_length(const Vector3D<T>& start, const Vector3D<T>& goal);
        T get_shortest_path_length(const Vector3D<T>& start, const Vector3D<T>& goal, Vector2D<T>& center_s_r, 
                Vector2D<T>& center_s_l, Vector2D<T>& center_g_r, Vector2D<T>& center_g_l);
        std::pair<T, bool> get_shortest_path(const Vector3D<T>& start, const Vector3D<T>& goal, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature);
        std::string get_path_type() const;

    private:
        // Private functions
        T get_params_rsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const;
        T get_params_rsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const;
        T get_params_lsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const;
        T get_params_lsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                const Vector3D<T>& start, const Vector3D<T>& goal, std::array<T, 4>& params) const;
        void sample_path_rsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const; 
        void sample_path_rsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const; 
        void sample_path_lsr(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const; 
        void sample_path_lsl(const Vector2D<T>& center_s, const Vector2D<T>& center_g, 
                std::vector<Vector3D<T>>& path, std::vector<T>& path_curvature) const;       

        // Private class members
        const T _r_min;                         // Minimum turning radius (m) of the Dubins car
        const T _step_size;                     // Step size (m) for sampling the Dubins paths
        const T _ang_step_size;                 // Angular step size (rad) for sampling Dubins path 
        std::array<T, 4> _params;               // Start angle and delta angle on starting circle and 
                                                // start angle and delta angle on goal circle of the shortest path
        Path _path_type;                        // Path lable determing the type of the shortest path (RLR and LRL neglected) 
    };
}

#endif