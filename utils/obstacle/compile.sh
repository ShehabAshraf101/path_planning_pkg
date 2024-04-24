#!/bin/bash

g++ -c ../../lib/Obstacle.cpp -o Obstacle.o -I../../include/path_planning_pkg -Wall

g++ -c test_obstacle.cpp -o test_obstacle.o -I../../include/path_planning_pkg -Wall

g++ Obstacle.o test_obstacle.o -o test_obstacle

./test_obstacle
