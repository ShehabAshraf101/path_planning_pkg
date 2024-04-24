#include <iostream>
#include "Obstacle.h"

using std::cout;
using planning::Obstacle;

int main(int argc, char **argv)
{
    // Initialize obstacle structs
    Obstacle<double> obstacle1(1.0, 0.5, 2.0, 1.0, 0.3, 0.5);
    Obstacle<float> obstacle2(3.0f, 5.0f, 0.25f, 0.25f);

    // Print out their parameters
    cout << "Obstacle 1" << std::endl;
    cout << "pos_x = " << obstacle1._pose2D._x << " pos_y = " << obstacle1._pose2D._y
        << " heading = " << obstacle1._pose2D._heading << std::endl;
    cout << "vel_x = " << obstacle1._velocity._x << " vel_y =" << obstacle1._velocity._y << std::endl;
    cout << "dim_x = " << obstacle1._dimensions._x << " dim_y = " << obstacle1._dimensions._y << std::endl;

    cout << "Obstacle 2" << std::endl;
    cout << "pos_x = " << obstacle2._pose2D._x << " pos_y = " << obstacle2._pose2D._y
        << " heading = " << obstacle2._pose2D._heading << std::endl;
    cout << "vel_x = " << obstacle2._velocity._x << " vel_y =" << obstacle2._velocity._y << std::endl;
    cout << "dim_x = " << obstacle2._dimensions._x << " dim_y = " << obstacle2._dimensions._y << std::endl;
    return 0;
}