#include "lightweight_robot_simulator/obstacles.hpp"
#include "lightweight_robot_simulator/obstacle_simulator.hpp"

using namespace lightweight_robot_simulator;

ObstacleSimulator::ObstacleSimulator(
    double obstacle_radius_, 
    double obstacle_moving_speed_)
  : obstacle_radius(obstacle_radius_),
    obstacle_moving_speed(obstacle_moving_speed_)
{
  /*
  * AF(obstacle_radius, obstacle_moving_speed) = A obstacle
  simulator that simulates obstacle with radius -> obstacle_radius and
  dynamic object moving with the speed of obstacle_moving_speed.
  
  * Rep:
    - static_obstacle_vector: a vector that stores all static obstacles. 
    - dynamic_obstacle_vector: a vector that stores all dynamic obstacles
  
  PUBLIC
  * add_static_object(x, y): add static object at pose location x, y
  * add_dynamic_object(x, y, mode): add dynamic object at pose location
    x, y moving with the pattern -> mode.
  * update_object_location(t): update object location at time t

  */
}

void ObstacleSimulator::add_static_object(double x, double y) {
  Obstacle obj = Obstacle(x, y, obstacle_radius);
  static_obstacle_vector.push_back(obj);
}

void ObstacleSimulator::add_dynamic_object(double x, double y, int mode) {
  DynamicObstacle obj = DynamicObstacle(x, y, obstacle_radius, obstacle_moving_speed, mode);
  dynamic_obstacle_vector.push_back(obj);
}

void ObstacleSimulator::update_object_location(double t) {
  
}


