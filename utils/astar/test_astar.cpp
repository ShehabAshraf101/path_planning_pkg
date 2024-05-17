#include <iostream>
#include <vector>
#include <utility>
#include <chrono>
#include "AStar.h"

using std::cout;
using std::endl;
using namespace planning;

int main(int argc, char **argv)
{
    // Initialize AStar with grid parameters
    float grid_resolution = 0.5f;
    float obstacle_threshold = 0.75f;
    float obstacle_prob_min = 0.1f;
    float obstacle_prob_max = 0.95f;
    float obstacle_prob_free = 0.4f;
    int grid_size = 60; // 60 x 60
    AStar<float> astar(grid_resolution, obstacle_threshold, obstacle_prob_min, 
            obstacle_prob_max, obstacle_prob_free, grid_size); // allows diagonal moves

    // Initialize start and goal locations as well as obstacles
    Vector2D<float> goal(25.5f, 36.0f);
    Vector2D<float> start(18.0f, 18.0f);
    std::pair<Vector2D<float>, Vector2D<float>> line1 {{21.9f, 4.5f}, {21.9f, 31.5f}};
    std::pair<Vector2D<float>, Vector2D<float>> line2 {{20.4f, 33.0f}, {38.4f, 33.0f}};
    std::pair<Vector2D<float>, Vector2D<float>> line3 {{10.5f, 4.5f}, {10.5f, 40.5f}};
    std::pair<Vector2D<float>, Vector2D<float>> line4 {{9.0f, 42.0f}, {39.0f, 42.0f}};
    // Obstacle<float> ob1(18.0f, 22.8f, 3.0f, 2.4f);
    // Obstacle<float> ob2(14.25f, 28.5f, 1.5f, 4.8f);
    // Obstacle<float> ob3(18.0f, 34.8f, 3.0f, 2.4f);
    // Obstacle<float> ob1(18.0f, 22.8f, 3.5f, 2.9f);
    // Obstacle<float> ob2(14.25f, 28.5f, 2.0f, 5.3f);
    // Obstacle<float> ob3(18.0f, 34.8f, 3.5f, 2.9f);
    Obstacle<float> ob1(18.0f, 22.8f, 4.0f, 3.4f);
    Obstacle<float> ob2(14.25f, 28.5f, 3.0f, 5.8f);
    Obstacle<float> ob3(18.0f, 34.8f, 4.0f, 3.4f);
    // std::vector<Obstacle<float>> obstacles; 
    // std::vector<std::pair<Vector2D<float>, Vector2D<float>>> lines; // No obstacles
    std::vector<std::pair<Vector2D<float>, Vector2D<float>>> lines {line1, line2, line3, line4}; // With drivable area only
    std::vector<Obstacle<float>> obstacles{ob1, ob2, ob3}; // Drivable area + obstacles

    // Test time for initializing obstacles
    Node2D<float> start_node(0, 0);
    astar.update_goal_start(goal, start, start_node);
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();
    for (std::size_t i = 0; i < 5; i++)
    {
        astar.update_obstacles();
        astar.update_obstacles(lines, std::vector<float>(lines.size(), 0.6f), 1.0f);
        astar.update_obstacles(obstacles, std::vector<float>(obstacles.size(), 0.75f));
    }
    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_seconds = end_time - start_time;
    cout << "Time for obstacle initialization = " << elapsed_seconds.count()/5 << "ms" << std::endl;

    // Solve using A*
    start_time = std::chrono::high_resolution_clock::now();
    float cost = astar.find_path(start_node._posd._x, start_node._posd._y);
    // float cost = astar.find_path(33, 36);
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Time for A* (first time) = " << elapsed_seconds.count() << "ms" << std::endl;
    cout << "Cost = " << cost << endl << endl;

    // Solve for node on explored path
    start_time = std::chrono::high_resolution_clock::now();
    cost = astar.find_path(33, 36);
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Time for A* (on path) = " << elapsed_seconds.count() << "ms" << std::endl;
    cout << "Cost = " << cost << endl << endl;

    // Solve for node close to start point
    start_time = std::chrono::high_resolution_clock::now();
    cost = astar.find_path(start_node._posd._x + 6, start_node._posd._y + 10);
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Time for A* (close to start) = " << elapsed_seconds.count() << "ms" << std::endl;
    cout << "Cost = " << cost << endl << endl;

    // Solve for path
    std::vector<Vector2D<float>> path;
    cost = astar.find_path(goal, start, path);
    cout << "Cost = " << cost << endl << endl;

    // Iterate over path in reverse and print out path from start to goal
    // cout << "path = [";
    // for (auto it = path.rbegin(); it != path.rend(); it++)
    // {
    //     cout << "[" << it->_x << ", " << it->_y << "]";
    //     if (it != (path.rend() - 1))
    //     {
    //         cout << " ,";
    //     }
    // }
    // cout << "]" << endl;

    // Print the obstacle map for debugging
    const std::vector<std::vector<float>>& obstacle_map = astar.get_obstacles();
    std::cout << "list = [";
    for (std::size_t i = 0; i < obstacle_map.size(); i++)
    {
        std::cout << "[";
        for (std::size_t j = 0; j < obstacle_map[i].size(); j++)
        {
            float prob = 1.0f - (1.0f/(1.0f + std::exp(obstacle_map[i][j]))); 
            std::cout << prob;
            if (j != (obstacle_map[i].size() - 1))
            {
                std::cout << ", ";
            }
        }
        std::cout << "]";
        if (i != (obstacle_map.size() - 1))
        {
            std::cout << ", \n";
        }
    }
    std::cout << "]" << "\n\n" << std::endl;

    return 0;
}