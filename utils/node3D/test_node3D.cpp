#include <iostream>
#include <vector>
#include <set>
#include <unordered_set>
#include "Node2D.h"
#include "Node3D.h"
#include "common.h"

using std::cout;
using namespace planning;

int main(int argc, char **argv)
{
    // Initialize some Node2D and Node3D objects
    Vector3D<double> pose2D(0.0, 0.0, 0.0);
    Node2D<double> base_node(3, 9, 3.4, 6.6, nullptr);
    Node3D<double> node1(pose2D, 0.0, 1.0, 3, 34, &base_node, nullptr);
    pose2D._x = 3.0;
    Node3D<double> node2(pose2D, 1.0, 25.0, 1, 23, &base_node, nullptr);
    pose2D._heading = M_PI_2;
    Node3D<double> node3(pose2D, 3.0, 4.0, 5, 70, &base_node, &node1);

    // Print out some of their values
    cout << node1 << std::endl;
    cout << node2 << std::endl;
    cout << node3 << std::endl;
    cout << "End of Test 1 (Constructing Nodes)" << std::endl;

    // Initialize a std::set of Node3D
    Node3D<double> node4(pose2D, 3.0, 2.0, 4, 34, &base_node, &node2);
    std::set<Node3D<double>> set_node;
    set_node.insert(node1);
    set_node.insert(node2);
    set_node.insert(node3);
    set_node.insert(node4);

    for(const auto& node : set_node)
    {
        cout << node << std::endl;
    }
    cout << "End of Test 2 (set of Nodes)" << std::endl;

    // Initialize a std::unordered_set of Node3D
    std::unordered_set<Node3D<double>, Node3D<double>::HashFunction> unordered_set_node;
    unordered_set_node.insert(node1);
    unordered_set_node.insert(node2);
    unordered_set_node.insert(node3);
    unordered_set_node.insert(node4);

    for (const auto& node : unordered_set_node)
    {
        cout << node << Node3D<double>::HashFunction{}(node) << std::endl;
    }
    cout << "End of Test 3 (unordered_set of Nodes)" << std::endl;

    // Try finding nodes in both sets
    auto it_set = set_node.find(node4);
    if (it_set == set_node.end())
    {
        cout << "Node not found in ordered set" << std::endl;
    }
    else
    {
        cout << "Node found in ordered set" << std::endl;
    }

    auto it_unset = unordered_set_node.find(node4);
    if (it_unset == unordered_set_node.end())
    {
        cout << "Node not found in unordered set" << std::endl;
    }
    else
    {
        cout << "Node found in unordered set" << std::endl;
    }

    return 0;
}