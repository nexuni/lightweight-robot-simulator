#include "lightweight_robot_simulator/obstacles.hpp"
#include "lightweight_robot_simulator/pose_2d.hpp"

using namespace lightweight_robot_simulator;

Obstacle::Obstacle(
    double x_, 
    double y_,
    double radius_)
  : x(x_), 
    y(y_), 
    radius(radius_)
{

  update_contour(x, y, radius);
}

void Obstacle::map_cell_tf(
    std::vector<int> & cell_indices, 
    size_t height_, 
    size_t width_, 
    double resolution_,
    const Pose2D & origin_) {

  // Assign parameters
  height = height_;
  width = width_;
  resolution = resolution_;
  origin = origin_;
  origin_c = std::cos(origin.theta);
  origin_s = std::sin(origin.theta);

  for (int i=0; i<contour.size(); i++){
    int cell_idx = xy_to_cell(contour[i].x, contour[i].y);
    cell_indices.push_back(cell_idx);
  }
}

void Obstacle::xy_to_row_col(double x, double y, int * row, int * col) const {
  // Translate the state by the origin
  double x_trans = x - origin.x;
  double y_trans = y - origin.y;

  // Rotate the state into the map
  double x_rot =   x_trans * origin_c + y_trans * origin_s;
  double y_rot = - x_trans * origin_s + y_trans * origin_c;

  // Clip the state to be a cell
  if (x_rot < 0 or x_rot >= width * resolution or
      y_rot < 0 or y_rot >= height * resolution) {
    *col = -1;
    *row = -1;
  } else {
    // Discretize the state into row and column
    *col = std::floor(x_rot/resolution);
    *row = std::floor(y_rot/resolution);
  }
}

int Obstacle::row_col_to_cell(int row, int col) const {
  return row * width + col;
}

int Obstacle::xy_to_cell(double x, double y) const {
  int row, col;
  xy_to_row_col(x, y, &row, &col);
  return row_col_to_cell(row, col);
}

void Obstacle::get_contour(std::vector<Point2D> & _contour){
  for (int i=0; i<contour.size(); i++){
    _contour.push_back(contour[i]);
  }
}

void Obstacle::update_contour(double x_input, double y_input, double radius_input){
  x = x_input;
  y = y_input;
  contour.clear();
  for (int i = 0; i <= obstacle_segments; i++) {
    double theta = (2 * M_PI * i)/((double) obstacle_segments);
    Point2D pt;
    pt.x = x_input + radius_input * std::cos(theta);
    pt.y = y_input + radius_input * std::sin(theta);
    contour.push_back(pt);
  }
}

DynamicObstacle::DynamicObstacle(
    double x_, 
    double y_,
    double theta_,
    double radius_,
    int mode_, 
    double speed_, 
    double init_time_)
  : Obstacle(x_, y_, radius_),
    theta(theta_), 
    mode(mode_), 
    speed(speed_),
    init_time(init_time_)
{
    last_update_time = init_time;
}

void DynamicObstacle::update_loc(double * nx, double * ny, double t)
{ 
    double dt = t - last_update_time;
    double time_elapsed = t - init_time;
    last_update_time = t;
    switch(mode){
      case 0:
        *nx = x + speed * dt * std::cos(theta);
        *ny = y + speed * dt * std::sin(theta);
        break;
      case 1:
        *nx = x + speed * dt * std::cos(theta + speed * time_elapsed / default_circle_radius);
        *ny = y + speed * dt * std::sin(theta + speed * time_elapsed / default_circle_radius);
        break;
      default:
        break;
    }
}

