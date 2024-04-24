#!/bin/bash

g++ -O3 -c ../../lib/Node2D.cpp -o Node2D.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Obstacle.cpp -o Obstacle.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Grid2D.cpp -o Grid2D.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/AStar.cpp -o AStar.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c test_astar.cpp -o test_astar.o -I../../include/path_planning_pkg -Wall

g++ -O3 Node2D.o Obstacle.o Grid2D.o AStar.o test_astar.o -o test_astar

./test_astar