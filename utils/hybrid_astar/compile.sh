#!/bin/bash

g++ -O3 -c ../../lib/HybridAStar.cpp -o HybridAStar.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/AStar.cpp -o AStar.o -I../../include/path_planning_pkg -DSTORE_GRID_AS_REFERENCE -Wall

g++ -O3 -c ../../lib/Dubins.cpp -o Dubins.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/VehicleModel.cpp -o VehicleModel.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Grid3D.cpp -o Grid3D.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Grid2D.cpp -o Grid2D.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Node3D.cpp -o Node3D.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Node2D.cpp -o Node2D.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Obstacle.cpp -o Obstacle.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/VelocityGenerator.cpp -o VelocityGenerator.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c test_hybrid_astar.cpp -o test_hybrid_astar.o -I../../include/path_planning_pkg -Wall

g++ -O3 HybridAStar.o AStar.o Dubins.o VehicleModel.o Grid3D.o Grid2D.o Node3D.o Node2D.o Obstacle.o VelocityGenerator.o test_hybrid_astar.o -o test_hybrid_astar

./test_hybrid_astar