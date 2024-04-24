#include <iostream>
#include <vector>
#include <set>
#include <unordered_set>
#include "Node2D.h"

using std::cout;
using namespace planning;

int main(int argc, char **argv)
{
    // Initialize some Node2D objects
    Node2D<float> node_f(1, 5);
    Node2D<double> node_d1(3, 9, 3.4, 6.6, nullptr);
    Node2D<double> node_d2(4, 10, 4.5, 5.5, &node_d1);

    // Print out some of their values
    cout << node_f << std::endl;
    cout << node_d1 << std::endl;
    cout << node_d2 << std::endl;
    cout << "End of Test 1 (Constructing Nodes)" << std::endl;

    // Initialize a 2D vector and a set of Node2D
    std::vector<std::vector<Node2D<double>>> vect(10, std::vector<Node2D<double>>(10, Node2D<double>(4, 4)));
    cout << vect[2][4] << std::endl;
    cout << "End of Test 2 (2D vector of Nodes)" << std::endl;
    
    Node2D<double> node_d3(4, 10, 5.0, 5.5, &node_d1);
    Node2D<double> node_d4(9, 3, 3.0, 5.0, &node_d1);
    std::set<Node2D<double>> set_node;
    set_node.insert(node_d1);
    set_node.insert(node_d2);
    set_node.insert(node_d3);
    set_node.insert(node_d4);

    for(const auto& node : set_node)
    {
        cout << node << std::endl;
    }
    cout << "End of Test 3 (set of Nodes)" << std::endl;

    // Initialize an unordered set of Node2D
    std::unordered_set<Node2D<double>, Node2D<double>::HashFunction> unordered_set_node;
    unordered_set_node.insert(node_d1);
    unordered_set_node.insert(node_d2);
    unordered_set_node.insert(node_d3);
    unordered_set_node.insert(node_d4);

    for (const auto& node : unordered_set_node)
    {
        cout << node << Node2D<double>::HashFunction{}(node) << std::endl;
    }
    cout << "End of Test 4 (unordered_set of Nodes)" << std::endl;

    // Try finding nodes in both sets
    auto it_set = set_node.find(node_d2);
    if (it_set == set_node.end())
    {
        cout << "Node not found in ordered set" << std::endl;
    }
    else
    {
        cout << "Node found in ordered set" << std::endl;
    }

    auto it_unset = unordered_set_node.find(node_d3);
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