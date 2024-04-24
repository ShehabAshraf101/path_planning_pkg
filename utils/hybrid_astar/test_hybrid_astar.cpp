#include <iostream>
#include <vector>
#include <utility>
#include <chrono>
#include "HybridAStar.h"

using std::cout;
using namespace planning;

int main(int argc, char **argv)
{
    // Initialize the 2D grid parameters
    float grid_resolution = 0.5f;
    int grid_size = 60; // 60 x 60
    bool grid_allow_diag_moves = true;

    // Initialize vehicle parameters
    float step_size = 0.75f;
    float max_lat_acc = 4.0f;
    float max_long_dec = 2.0f;
    float wheelbase = 2.269f;
    float rear_to_cg = 1.1f;

    // Initialize 3D grid parameters
    int num_angle_bins = 72;
    int num_actions = 1;
    // std::vector<float> curvature_weights{1.5, 2, 2, 2, 2, 2, 1.5};
    // std::vector<float> steering{-45, -30, -15, 0, 15, 30, 45};
    std::vector<float> curvature_weights{1.0, 2.0, 0, 2.0, 1.0};
    std::vector<float> steering{-40, -20, 0, 20, 40};
    for (auto& angle : steering)
    {
        angle = angle * M_PI/180.0f;
    }

    // Initialize Dubins shot parameters
    int dubins_shot_interval = 250;
    int dubins_shot_interval_decay = 10;

    // Initialize obstacles
    Obstacle<float> ob1(21.9f, 18.0f, 3.0f, 27.0f);
    Obstacle<float> ob2(29.4f, 33.0f, 18.0f, 3.0f);
    Obstacle<float> ob3(10.5f, 22.5f, 3.0f, 36.0f);
    Obstacle<float> ob4(24.0f, 42.0f, 30.0f, 3.0f);
    Obstacle<float> ob5(18.0f, 22.8f, 3.0f, 2.4f);
    Obstacle<float> ob6(14.25f, 28.5f, 1.5f, 4.8f);
    Obstacle<float> ob7(18.0f, 34.8f, 3.0f, 2.4f);
    // std::vector<Obstacle<float>> obstacles; // No obstacles
    // std::vector<Obstacle<float>> obstacles{ob1, ob2, ob3, ob4}; // With drivable area only
    std::vector<Obstacle<float>> obstacles{ob1, ob2, ob3, ob4, ob5, ob6, ob7}; // Drivable area + obstacles

    // Initialize goal and start 2D poses
    Vector3D<float> start(18.0f, 18.0f, M_PI_2);
    Vector3D<float> goal(26.0f, 36.0f, 0.0f);
    float vel_start = 2.0;

    // Initialize chrono objects for timing operations
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time;
    std::chrono::duration<double, std::milli> elapsed_seconds;

    // Initialize Hybrid A* object with parameters
    HybridAStar<float> hybrid_astar(dubins_shot_interval, dubins_shot_interval_decay, grid_resolution, 
            grid_size, grid_allow_diag_moves, step_size, max_lat_acc, max_long_dec, wheelbase, 
            rear_to_cg, num_angle_bins, num_actions, steering, curvature_weights);

    
    // Search for collision free and feasible path using Hybrid A*
    std::vector<float> curvature;
    std::vector<Vector3D<float>> path;
    start_time = std::chrono::high_resolution_clock::now();
    std::pair<float, bool> result = hybrid_astar.find_path(vel_start, goal, start, obstacles, path, curvature);
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Hybrid A* success = " << result.second << std::endl;
    cout << "Hybrid A* cost = " << result.first << std::endl;
    cout << "Hybrid A* execution time = " << elapsed_seconds.count() << "ms" << std::endl;

    // Print out the returned path
    cout << "path = np.array([";
    for (auto it = path.rbegin(); it != path.rend(); it++)
    {
        cout << "[" << it->_x << ", " << it->_y << ", " << it->_heading << "]";
        if ((it + 1) != path.rend())
        {
            cout << ", ";
        }
    }
    cout << "])" << std::endl;
}