#pragma once

#include <random>

#include "lightweight_robot_simulator/obstacles.hpp"
#include "lightweight_robot_simulator/obstacle_simulator.hpp"

namespace lightweight_robot_simulator {

class ObstacleSimulator {

  private:

    double obstacle_radius;
    double obstacle_moving_speed;

    std::vector<Obstacle> static_obstacle_vector;
    std::vector<DynamicObstacle> dynamic_obstacle_vector;

  public:
    ObstacleSimulator() {}

    ObstacleSimulator(
        double obstacle_radius_, 
        double obstacle_moving_speed_);

    void add_static_object(double x, double y);
    void add_dynamic_object(double x, double y, int mode);
    void update_object_location(double t);
};

}
