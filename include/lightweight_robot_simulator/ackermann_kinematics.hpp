#pragma once

#include "lightweight_robot_simulator/pose_2d.hpp"

namespace lightweight_robot_simulator {

class AckermannKinematics {

  public:

    static double angular_velocity(
        double velocity,
        double steering_angle,
        double wheelbase);

    static Pose2D update(
        const Pose2D start, 
        double velocity, 
        double steering_angle, 
        double wheelbase, 
        double dt);

};

}
