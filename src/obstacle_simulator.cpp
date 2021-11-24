#include "lightweight_robot_simulator/pose_2d.hpp"
#include "lightweight_robot_simulator/obstacles.hpp"
#include "lightweight_robot_simulator/obstacle_simulator.hpp"

using namespace lightweight_robot_simulator;

ObstacleSimulator::ObstacleSimulator(
    double obstacle_radius_, 
    double obstacle_moving_speed_, 
    Pose2D & origin_, 
    size_t map_width_,
    size_t map_height_,
    double map_resolution_)
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
  min_x = origin_.x + 2*obstacle_radius;
  max_x = origin_.x + map_width_ * map_resolution_ - 2*obstacle_radius;

  min_y = origin_.y + 2*obstacle_radius;
  max_y = origin_.y + map_height_ * map_resolution_ - 2*obstacle_radius;
}

void ObstacleSimulator::add_static_object(double x, double y) {
  Obstacle obj;
  obj = Obstacle(x, y, obstacle_radius);
  static_obstacle_vector.push_back(obj);
}

void ObstacleSimulator::add_dynamic_object(double x, double y, double theta, int mode, double time) {
  DynamicObstacle obj = DynamicObstacle(x, y, theta, obstacle_radius, mode, obstacle_moving_speed, time);
  dynamic_obstacle_vector.push_back(obj);
}

void ObstacleSimulator::update_object_location(double t) {
  for (int i=0; i<dynamic_obstacle_vector.size(); i++){
    double nx, ny;
    dynamic_obstacle_vector[i].update_loc(&nx, &ny, t);

    // Cap boundaries
    nx = std::min(std::max(nx, min_x), max_x);
    ny = std::min(std::max(ny, min_y), max_y);
    dynamic_obstacle_vector[i].update_contour(nx, ny, obstacle_radius);
  }
}


