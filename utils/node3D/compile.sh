#!/bin/bash

g++ -c ../../lib/Node2D.cpp -o Node2D.o -I../../include/path_planning_pkg -Wall

g++ -c ../../lib/Node3D.cpp -o Node3D.o -I../../include/path_planning_pkg -Wall

g++ -c test_node3D.cpp -o test_node3D.o -I../../include/path_planning_pkg -Wall

g++ Node2D.o Node3D.o test_node3D.o -o test_node3D

./test_node3D