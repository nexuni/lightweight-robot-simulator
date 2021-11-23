#pragma once

#include <random>
#include "lightweight_robot_simulator/pose_2d.hpp"

namespace lightweight_robot_simulator {

struct Point2D {
    double x;
    double y;
};

class Obstacle {

  protected:
    double obstacle_segments = 24; 
    // how many segments a circle to be divided
    std::vector<Point2D> contour;

    double resolution;
    size_t width, height;
    Pose2D origin;
	// Precomputed constants
    double origin_c;
    double origin_s;

  public:
    Obstacle() {}

    Obstacle(
    	double x_, 
    	double y_,
      double radius_);

    double x, y;
    double radius;

    void map_cell_tf(
        std::vector<int> & cell_indices, 
        size_t height, 
        size_t width, 
        double resolution,
        const Pose2D & origin);

   	void xy_to_row_col(double x, double y, int * row, int * col) const;
    int row_col_to_cell(int row, int col) const;
    int xy_to_cell(double x, double y) const;
    void update_contour(double x, double y, double radius);
    void get_contour(std::vector<Point2D> & _contour);
};

class DynamicObstacle : public Obstacle{

  protected:
    double speed;
    double theta;
    double init_time, last_update_time;
    int mode; // 0: back and forth, 1: circle
    double default_circle_radius = 2.0;

  public:
    DynamicObstacle() {}

    DynamicObstacle(
        double x_,
        double y_,
        double theta_,
        double radius_,
        int mode_, 
        double speed_,
        double init_time_);

    void update_loc(double * nx, double * ny, double t);
};

}