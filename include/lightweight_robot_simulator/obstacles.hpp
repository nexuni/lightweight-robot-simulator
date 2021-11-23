/*
  * obstacle_to_cell_tf(*obstacle_cell_vector, map): This is the function to 
  transform obstacle to cell location and save it in a vector. 
*/

#pragma once

namespace lightweight_robot_simulator {

struct Point2D {
    double x;
    double y;
};

class Obstacle {

  protected:

  	double x, y;
    double radius;
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
      int radius_);

    void map_cell_tf(
        std::vector<int> & cell_indices, 
        size_t height, 
        size_t width, 
        double resolution,
        const Pose2D & origin);

   	void xy_to_row_col(double x, double y, int * row, int * col) const;
    int row_col_to_cell(int row, int col) const;
    int xy_to_cell(double x, double y) const;
    void get_contour(std::vector<Point2D> * contour);
};

class DynamicObstacle : public Obstacle{

  protected:
    double speed;
    int mode; // 0: back and forth, 1: circle

  public:
    DynamicObstacle() {}

    DynamicObstacle(
        double x_,
        double y_,
        int radius_,
        int mode_, 
        double speed_);

    void get_loc(double * x, double * y, double t) const;
};

}