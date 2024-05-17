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
    float obstacle_threshold = 0.75f;
    float obstacle_prob_min = 0.1f;
    float obstacle_prob_max = 0.95f;
    float obstacle_prob_free = 0.4f;
    int grid_size = 60; // 60 x 60
    bool grid_allow_diag_moves = true;

    // Initialize vehicle parameters
    float step_size = 0.75f;
    float max_lat_acc = 4.0f;
    float max_long_dec = 2.0f;
    float wheelbase = 2.269f;
    float rear_to_cg = 1.1f;

    // Initialize 3D grid parameters
    float apf_rep_constant = 1.0f;
    float apf_active_angle = static_cast<float>(M_PI/4);
    int num_angle_bins = 72;
    int num_actions = 1;
    std::vector<float> curvature_weights{0, 0, 0, 0, 0, 0, 0};
    // std::vector<float> steering{-40, -20, 0, 20, 40};
    std::vector<float> steering{-30, -15, 0, 15, 30};
    for (auto& angle : steering)
    {
        angle = angle * M_PI/180.0f;
    }

    // Initialize Dubins shot parameters
    int dubins_shot_interval = 300;
    int dubins_shot_interval_decay = 10;

    // Initialize obstacles
    std::pair<Vector2D<float>, Vector2D<float>> line1 {{21.9f, 4.5f}, {21.9f, 31.5f}};
    std::pair<Vector2D<float>, Vector2D<float>> line2 {{20.4f, 33.0f}, {38.4f, 33.0f}};
    std::pair<Vector2D<float>, Vector2D<float>> line3 {{10.5f, 4.5f}, {10.5f, 40.5f}};
    std::pair<Vector2D<float>, Vector2D<float>> line4 {{9.0f, 42.0f}, {39.0f, 42.0f}};
    // Obstacle<float> ob1(18.0f, 22.8f, 3.0f, 2.4f);
    // Obstacle<float> ob2(14.25f, 28.5f, 1.5f, 4.8f);
    // Obstacle<float> ob3(18.0f, 34.8f, 3.0f, 2.4f);
    Obstacle<float> ob1(18.0f, 22.8f, 3.5f, 2.9f);
    Obstacle<float> ob2(14.25f, 28.5f, 2.0f, 5.3f);
    Obstacle<float> ob3(18.0f, 34.8f, 3.5f, 2.9f);
    // Obstacle<float> ob1(18.0f, 22.8f, 4.0f, 3.4f);
    // Obstacle<float> ob2(14.25f, 28.5f, 3.0f, 5.8f);
    // Obstacle<float> ob3(18.0f, 34.8f, 4.0f, 3.4f);
    // std::vector<Obstacle<float>> obstacles; 
    // std::vector<std::pair<Vector2D<float>, Vector2D<float>>> lines; // No obstacles
    std::vector<std::pair<Vector2D<float>, Vector2D<float>>> lines {line1, line2, line3, line4}; // With drivable area only
    std::vector<Obstacle<float>> obstacles{ob1, ob2, ob3}; // Drivable area + obstacle

    // Initialize goal and start 2D poses
    Vector3D<float> start(18.0f, 18.0f, M_PI_2);
    Vector3D<float> goal(26.0f, 36.0f, 0.0f);
    float vel_start = 2.0f;

    // Initialize chrono objects for timing operations
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time;
    std::chrono::duration<float, std::milli> elapsed_seconds;

    // Initialize Hybrid A* object with parameters
    HybridAStar<float> hybrid_astar(dubins_shot_interval, dubins_shot_interval_decay, grid_resolution, 
            obstacle_threshold, obstacle_prob_min, obstacle_prob_max, obstacle_prob_free,
            grid_size, grid_allow_diag_moves, step_size, max_lat_acc, max_long_dec, wheelbase, 
            rear_to_cg, apf_rep_constant, apf_active_angle, num_angle_bins, num_actions, 
            steering, curvature_weights);

    
    // Initialize the goal node as well as obstacles
    hybrid_astar.update_goal(goal, start);
    start_time = std::chrono::high_resolution_clock::now();
    for (std::size_t i = 0; i < 5; i++)
    {
        hybrid_astar.update_obstacles();
        hybrid_astar.update_obstacles(lines, std::vector<float>(lines.size(), 0.6f), 1.25f);
        hybrid_astar.update_obstacles(obstacles, std::vector<float>(obstacles.size(), 0.75f), 2.5f);
    }
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Time for obstacle initialization = " << elapsed_seconds.count()/5 << "ms" << std::endl;

    // Search for collision free and feasible path using Hybrid A*
    std::vector<float> curvature;
    std::vector<Vector3D<float>> path;
    start_time = std::chrono::high_resolution_clock::now();
    std::pair<float, bool> result = hybrid_astar.find_path(vel_start, start, path, curvature);
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Hybrid A* success = " << result.second << std::endl;
    cout << "Hybrid A* cost = " << result.first << std::endl;
    cout << "Hybrid A* execution time = " << elapsed_seconds.count() << "ms" << std::endl;

    // Print out the returned path
    int count = 0;
    cout << "path = np.array([";
    for (auto it = path.rbegin(); it != path.rend(); it++)
    {
        cout << "[" << it->_x << ", " << it->_y << ", " << it->_heading << "]";
        if ((it + 1) != path.rend())
        {
            cout << ", ";
            count++;
            if ((count % 10) == 0)
            {
                cout << "\n";
            }
        }
    }
    cout << "])" << std::endl;
}