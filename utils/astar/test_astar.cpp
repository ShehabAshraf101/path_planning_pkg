#include <iostream>
#include <vector>
#include <chrono>
#include "AStar.h"

using std::cout;
using std::endl;
using namespace planning;

int main(int argc, char **argv)
{
    // Initialize AStar with grid parameters
    float grid_resolution = 0.5f;
    int grid_size = 60; // 60 x 60
    AStar<float> astar(grid_resolution, grid_size); // allows diagonal moves

    // Initialize start and goal locations as well as obstacles
    Vector2D<float> goal(25.5f, 36.0f);
    Vector2D<float> start(18.0f, 18.0f);
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

    // Test time for initializing obstacles
    Node2D<float> start_node(0, 0);
    astar.update_goal_start(goal, start, start_node);
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();
    astar.update_obstacles(obstacles);
    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_seconds = end_time - start_time;
    cout << "Time for obstacle initialization = " << elapsed_seconds.count() << "ms" << std::endl;

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
    // std::vector<Vector2D<float>> path;
    // float cost = astar.find_path(goal, start, obstacles, path);
    // cout << "Cost = " << cost << endl << endl;

    // // Iterate over path in reverse and print out path from start to goal
    // cout << "path = [";
    // for (auto it = path.rbegin(); it != path.rend(); it++)
    // {
    //     cout << "[" << it->_x << ", " << it->_y << "], ";
    // }
    // cout << "]" << endl;

    return 0;
}