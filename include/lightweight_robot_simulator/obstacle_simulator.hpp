#pragma once

#include <random>

#include "lightweight_robot_simulator/obstacles.hpp"
#include "lightweight_robot_simulator/obstacle_simulator.hpp"

namespace lightweight_robot_simulator {

class ObstacleSimulator {

  private:

    double obstacle_radius;
    double obstacle_moving_speed;
    double min_x, min_y, max_x, max_y;

    std::vector<Obstacle> static_obstacle_vector;
    std::vector<DynamicObstacle> dynamic_obstacle_vector;

  public:
    ObstacleSimulator() {}

    ObstacleSimulator(
        double obstacle_radius_, 
        double obstacle_moving_speed_,
        Pose2D & origin_, 
        size_t map_width_,
        size_t map_height_,
        double map_resolution_);

    std::vector<Obstacle> get_static_obstacles() {return static_obstacle_vector;}
    std::vector<DynamicObstacle> get_dynamic_obstacles() {return dynamic_obstacle_vector;}

    void add_static_object(double x, double y);
    void add_dynamic_object(double x, double y, double theta, int mode, double time);
    void update_object_location(double t);
};

}
