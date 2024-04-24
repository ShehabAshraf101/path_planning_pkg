#!/bin/bash

g++ -O3 -c ../../lib/Node2D.cpp -o Node2D.o -I../../include/path_planning_pkg -Wall

g++ -O3 -c ../../lib/Node3D.cpp -o Node3D.o -I../../include/path_planning_pkg -Wall
 
g++ -O3 -c ../../lib/Dubins.cpp -o Dubins.o -I../../include/path_planning_pkg -Wall
 
g++ -O3 -c ../../lib/VehicleModel.cpp -o VehicleModel.o -I../../include/path_planning_pkg -Wall
 
g++ -O3 -c test_vehicle_dubins.cpp -o test_vehicle_dubins.o -I../../include/path_planning_pkg -Wall

g++ -O3 Node2D.o Node3D.o Dubins.o VehicleModel.o test_vehicle_dubins.o -o test_vehicle_dubins

./test_vehicle_dubins