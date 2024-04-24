#!/bin/bash

g++ -c ../../lib/Node2D.cpp -o Node2D.o -I../../include/path_planning_pkg -Wall

g++ -c test_node2D.cpp -o test_node2D.o -I../../include/path_planning_pkg -Wall

g++ Node2D.o test_node2D.o -o test_node2D

./test_node2D
