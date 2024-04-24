#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <utility>
#include <string>
#include "Dubins.h"
#include "VehicleModel.h"

using std::cout;
using std::endl;
using namespace planning;

int main(int argc, char **argv)
{
    // Define vehicle parameters
    double step_size = 0.5;
    double wheelbase = 2.269;
    double rear_to_cg = 1.1;
    double max_steering = 30.0 * M_PI/180.0;
    double max_beta = std::atan2(rear_to_cg * std::tan(max_steering), wheelbase);
    double rmin = wheelbase/(std::tan(max_steering) * std::cos(max_beta));
    std::vector<double> steering{-30.0 * M_PI/180.0, -20.0 * M_PI/180.0, -10.0 * M_PI/180.0,
                                0.0, 10.0 * M_PI/180.0, 20.0 * M_PI/180.0, 30.0 * M_PI/180.0};
    cout << "Minimum turning radius = " << rmin << "(m)" << endl;
    
    // Test the Dubins class for calculating the path from a start to a goal pose
    std::chrono::time_point<std::chrono::steady_clock> start_time, end_time;
    Vector3D<double> start(0.0, 0.0, 0.0);
    Vector3D<double> goal(20.0, -20.0, M_PI_2);
    std::vector<Vector3D<double>> path;
    std::vector<double> path_curvature;
    Dubins<double> dubins(rmin, step_size);
    dubins.get_shortest_path_length(start, goal);
    start_time = std::chrono::steady_clock::now();
    double shortest_path = dubins.get_shortest_path_length(start, goal);
    end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed_seconds = end_time - start_time;
    cout << "Time for calculating shortest path length = " << elapsed_seconds.count() << "ms" << endl;
    cout << "Shortest path from start to goal pose = " << shortest_path << "(m)" << endl;
    cout << "Shortest path type = " << dubins.get_path_type() << endl;

    start_time = std::chrono::steady_clock::now();
    dubins.get_shortest_path(start, goal, path, path_curvature);
    end_time = std::chrono::steady_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Time for getting shortest Dubins path = " << elapsed_seconds.count() << "ms" << endl;
    
    // cout << "path = np.array([";
    // for (int i = 0; i < path.size(); i++)
    // {
    //     cout << "[" << path[i]._x << ", " << path[i]._y << ", " << path[i]._heading << "]";
    //     if (i != (path.size() - 1))
    //     {
    //         cout << ", ";
    //     }
    // }
    // cout << "])" << endl;

    // Test vehicle model for a set of inputs
    std::vector<int> input_indices{6, 6, 6, 6, 6, 6, 
                              5, 5, 5, 5, 4, 4, 4, 4, 4,
                              3, 3, 3, 3, 3, 4, 4, 4, 4,
                              5, 5, 5, 5, 4, 4, 4, 4};
    start._heading = 0.0;
    Node3D<double> start_node(start, 0.0, 16.0, 3, get_heading_index(0.0, 5.0*M_PI/180.0), nullptr);
    VehicleModel<double> model(step_size, 4.0, 2.0, wheelbase, rear_to_cg, 72, 1, steering);
    Node3D<double> next_node = start_node;

    // cout << "path = np.array([";
    // cout << "[" << next_node._pose2D._x << ", " << next_node._pose2D._y << "], ";
    // for (const auto& index : input_indices)
    // {
    //     std::pair<bool, Node3D<double>> result = model.simulate_action(next_node, index);
    //     if (result.first)
    //     {
    //         cout << "[" << result.second._pose2D._x << ", " << result.second._pose2D._y << "], ";
    //         next_node = result.second;
    //     }
    //     else
    //     {
    //         cout << endl << "Infeasible motion" << endl;
    //         break;
    //     }
    // }
    // cout << "])" << endl;

    // Test execution time of model
    start_time = std::chrono::steady_clock::now();
    model.simulate_action(start_node, input_indices[0]);
    end_time = std::chrono::steady_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Time for simulating an action = " << elapsed_seconds.count() << "ms" << endl;

    start_time = std::chrono::steady_clock::now();
    std::vector<Node3D<double>> neighbors;
    model.get_neighbors(start_node, neighbors);
    end_time = std::chrono::steady_clock::now();
    elapsed_seconds = end_time - start_time;
    cout << "Num of neighbors = " << neighbors.size() << endl;
    cout << "Time for retrieving neighbors = " << elapsed_seconds.count() << "ms" << endl;
    
    return 0;
}