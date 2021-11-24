#include <ros/ros.h>
#include <ros/console.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include "lightweight_robot_simulator/pose_2d.hpp"
#include "lightweight_robot_simulator/ackermann_kinematics.hpp"
#include "lightweight_robot_simulator/scan_simulator_2d.hpp"
#include "lightweight_robot_simulator/obstacle_simulator.hpp"

using namespace lightweight_robot_simulator;

class RobotSimulator {
  private:
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame, scan2_frame;

    // The car state and parameters
    Pose2D pose;
    double wheelbase;
    double speed;
    double steering_angle;
    double previous_seconds;
    double scan_distance_to_base_link;
    double scan2_distance_to_base_link;
    double max_speed, max_steering_angle;
    double obstacle_radius, obstacle_moving_speed;
    int dynamic_obstacle_mode;

    // A simulator of the laser
    ScanSimulator2D scan_simulator, scan2_simulator;
    ObstacleSimulator obstacle_simulator;
    double map_free_threshold;

    // Joystick parameters
    int joy_speed_axis, joy_angle_axis;
    double joy_max_speed;

    // A ROS node
    ros::NodeHandle n;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    ros::Timer update_pose_timer, update_obstacle_timer;

    // Listen for drive and joystick commands
    ros::Subscriber drive_sub;
    ros::Subscriber joy_sub;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    ros::Subscriber pose_sub;
    ros::Subscriber pose_rviz_sub;
    ros::Subscriber obstacle_rviz_sub;
    ros::Subscriber dynamic_obstacle_rviz_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    ros::Publisher scan_pub;
    ros::Publisher scan2_pub;
    ros::Publisher odom_pub;
    ros::Publisher obstacle_pub;

    // Noise to add to odom sensor
    std::mt19937 noise_generator;    
    std::normal_distribution<double> twist_x_lin_dist, twist_x_ang_dist,
      twist_y_ang_dist, twist_z_ang_dist;
    std::vector<double> mu_twist;

  public:

    RobotSimulator() {
      // Initialize the node handle
      n = ros::NodeHandle("~");

      // Initialize the pose and driving commands
      pose = {.x=0, .y=0, .theta=0};
      speed = 0;
      steering_angle = 0;
      previous_seconds = ros::Time::now().toSec();

      // Get the topic names
      std::string joy_topic, drive_topic, map_topic, 
        scan_topic, pose_topic, pose_rviz_topic, odom_topic, scan2_topic, 
        static_obstacle_topic, obstacle_pub_topic, dynamic_obstacle_topic;
      n.getParam("joy_topic", joy_topic);
      n.getParam("drive_topic", drive_topic);
      n.getParam("map_topic", map_topic);
      n.getParam("scan_topic", scan_topic);
      n.getParam("scan2_topic", scan2_topic);
      n.getParam("pose_topic", pose_topic);
      n.getParam("odom_topic", odom_topic);
      n.getParam("pose_rviz_topic", pose_rviz_topic);
      n.getParam("static_obstacle_topic", static_obstacle_topic);
      n.getParam("obstacle_pub_topic", obstacle_pub_topic);
      n.getParam("dynamic_obstacle_topic", dynamic_obstacle_topic);

      // Get the transformation frame names
      n.getParam("map_frame", map_frame);
      n.getParam("base_frame", base_frame);
      n.getParam("scan_frame", scan_frame);
      n.getParam("scan2_frame", scan2_frame);

      // Fetch the car parameters
      int scan_beams;
      double update_pose_rate, scan_field_of_view, scan_std_dev;
      n.getParam("wheelbase", wheelbase);
      n.getParam("update_pose_rate", update_pose_rate);
      n.getParam("scan_beams", scan_beams);
      n.getParam("scan_field_of_view", scan_field_of_view);
      n.getParam("scan_std_dev", scan_std_dev);
      n.getParam("map_free_threshold", map_free_threshold);
      n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
      n.getParam("scan2_distance_to_base_link", scan2_distance_to_base_link);
      n.getParam("max_speed", max_speed);
      n.getParam("max_steering_angle", max_steering_angle);

      // Get joystick parameters
      bool joy;
      n.getParam("joy", joy);
      n.getParam("joy_speed_axis", joy_speed_axis);
      n.getParam("joy_angle_axis", joy_angle_axis);
      n.getParam("joy_max_speed", joy_max_speed);

      // Determine if we should broadcast
      n.getParam("broadcast_transform", broadcast_transform);

      // Obstacle parameters
      n.getParam("obstacle_radius", obstacle_radius);
      n.getParam("obstacle_moving_speed", obstacle_moving_speed);

      n.getParam("dynamic_obstacle_mode", dynamic_obstacle_mode);

      // Initialize a simulator of the laser scanner
      scan_simulator = ScanSimulator2D(
          scan_beams,
          scan_field_of_view,
          scan_std_dev);

      scan2_simulator = ScanSimulator2D(
          scan_beams,
          scan_field_of_view,
          scan_std_dev);


      // Noise to add to Odometry twist
      n.getParam("mu_twist", mu_twist);
      noise_generator = std::mt19937(std::random_device{}());
      twist_x_lin_dist = std::normal_distribution<double>(0., mu_twist[0]);
      twist_x_ang_dist = std::normal_distribution<double>(0., mu_twist[3]);
      twist_y_ang_dist = std::normal_distribution<double>(0., mu_twist[4]);
      twist_z_ang_dist = std::normal_distribution<double>(0., mu_twist[5]);

      // Make a publisher for laser scan messages
      scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
      scan2_pub = n.advertise<sensor_msgs::LaserScan>(scan2_topic, 1);

      // Make a publisher for odometry messages
      odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

      obstacle_pub = n.advertise<geometry_msgs::PolygonStamped>(obstacle_pub_topic, 1, true);

      // Start a timer to output the pose
      update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RobotSimulator::update_pose, this);

      update_obstacle_timer = n.createTimer(ros::Duration(update_pose_rate), &RobotSimulator::update_obstacle, this);

      // If the joystick is enabled
      if (joy)
        // Start a subscriber to listen to joystick commands
        joy_sub = n.subscribe(joy_topic, 1, &RobotSimulator::joy_callback, this);

      // Start a subscriber to listen to drive commands
      drive_sub = n.subscribe(drive_topic, 1, &RobotSimulator::drive_callback, this);

      // Start a subscriber to listen to new maps
      map_sub = n.subscribe(map_topic, 1, &RobotSimulator::map_callback, this);

      obstacle_rviz_sub = n.subscribe(static_obstacle_topic, 1, &RobotSimulator::obstacle_callback, this);
      dynamic_obstacle_rviz_sub = n.subscribe(dynamic_obstacle_topic, 1, &RobotSimulator::dynamic_obstacle_callback, this);


      // Start a subscriber to listen to pose messages
      pose_sub = n.subscribe(pose_topic, 1, &RobotSimulator::pose_callback, this);
      pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &RobotSimulator::pose_rviz_callback, this);
    }

    void update_obstacle(const ros::TimerEvent&){
      obstacle_simulator.update_object_location(ros::Time::now().toSec());
      std::vector<Obstacle> sobs = obstacle_simulator.get_static_obstacles();
      std::vector<DynamicObstacle> dobs = obstacle_simulator.get_dynamic_obstacles();

      // Need to set obstacles in this order
      scan_simulator.set_dynamic_obstacles(dobs);
      scan_simulator.set_obstacles(sobs);
      
      // Need to set obstacles in this order
      scan2_simulator.set_dynamic_obstacles(dobs);
      scan2_simulator.set_obstacles(sobs);

      if (dobs.size() > 0){
        DynamicObstacle last = dobs.back();
        publish_obstacle(last.x, last.y, last.radius);
      }
      
    }

    void update_pose(const ros::TimerEvent&) {

      // Update the pose
      ros::Time timestamp = ros::Time::now();
      double current_seconds = timestamp.toSec();
      pose = AckermannKinematics::update(
          pose, 
          speed,
          steering_angle,
          wheelbase,
          current_seconds - previous_seconds);
      previous_seconds = current_seconds;

      // Convert the pose into a transformation
      geometry_msgs::Transform t;
      t.translation.x = pose.x;
      t.translation.y = pose.y;
      tf2::Quaternion quat;
      quat.setEuler(0., 0., pose.theta);
      t.rotation.x = quat.x();
      t.rotation.y = quat.y();
      t.rotation.z = quat.z();
      t.rotation.w = quat.w();

      // Add a header to the transformation
      geometry_msgs::TransformStamped ts;
      ts.transform = t;
      ts.header.stamp = timestamp;
      ts.header.frame_id = map_frame;
      ts.child_frame_id = base_frame;

      // Make an odom message as well
      nav_msgs::Odometry odom;
      odom.header.stamp = timestamp;
      odom.header.frame_id = map_frame;
      odom.child_frame_id = base_frame;
      odom.pose.pose.position.x = pose.x;
      odom.pose.pose.position.y = pose.y;
      odom.pose.pose.orientation.x = quat.x();
      odom.pose.pose.orientation.y = quat.y();
      odom.pose.pose.orientation.z = quat.z();
      odom.pose.pose.orientation.w = quat.w();
      odom.twist.twist.linear.x = speed;
      odom.twist.twist.angular.z = 
        AckermannKinematics::angular_velocity(speed, steering_angle, wheelbase);
        
      // Add noise to odom
      if (mu_twist[0] > 0) {
          odom.twist.twist.linear.x += twist_x_lin_dist(noise_generator);
          odom.twist.twist.angular.x += twist_x_ang_dist(noise_generator);
          odom.twist.twist.angular.x += twist_x_ang_dist(noise_generator);
          odom.twist.twist.angular.x += twist_x_ang_dist(noise_generator);
      }

      // Publish them
      if (broadcast_transform) br.sendTransform(ts);
      odom_pub.publish(odom);
      // Set the steering angle to make the wheels move
      set_steering_angle(steering_angle, timestamp);

      // If we have a map, perform a scan
      if (map_exists) {
        // Get the pose of the lidar, given the pose of base link
        // (base link is the center of the rear axle)
        Pose2D scan_pose;
        scan_pose.x = pose.x + scan_distance_to_base_link * std::cos(pose.theta);
        scan_pose.y = pose.y + scan_distance_to_base_link * std::sin(pose.theta);
        scan_pose.theta = pose.theta;

        // Compute the scan from the lidar
        std::vector<double> scan = scan_simulator.scan(scan_pose);

        // Convert to float
        std::vector<float> scan_(scan.size());
        for (size_t i = 0; i < scan.size(); i++)
          scan_[i] = scan[i];

        // Publish the laser message
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = scan_frame;
        scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_increment = scan_simulator.get_angle_increment();
        scan_msg.range_max = 100;
        scan_msg.ranges = scan_;
        scan_msg.intensities = scan_;

        scan_pub.publish(scan_msg);

        // Publish a transformation between base link and laser
        geometry_msgs::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = timestamp;
        scan_ts.header.frame_id = base_frame;
        scan_ts.child_frame_id = scan_frame;
        br.sendTransform(scan_ts);

        // Do it for a second scan
        // Get the pose of the lidar, given the pose of base link
        // (base link is the center of the rear axle)
        Pose2D scan2_pose;
        scan2_pose.x = pose.x + scan2_distance_to_base_link * std::cos(pose.theta);
        scan2_pose.y = pose.y + scan2_distance_to_base_link * std::sin(pose.theta);
        scan2_pose.theta = pose.theta + 3.14159;

        // Compute the scan from the lidar
        std::vector<double> scan2 = scan2_simulator.scan(scan2_pose);

        // Convert to float
        std::vector<float> scan2_(scan2.size());
        for (size_t i = 0; i < scan2.size(); i++)
          scan2_[i] = scan2[i];

        // Publish the laser message
        sensor_msgs::LaserScan scan2_msg;
        scan2_msg.header.stamp = timestamp;
        scan2_msg.header.frame_id = scan2_frame;
        scan2_msg.angle_min = -scan2_simulator.get_field_of_view()/2.;
        scan2_msg.angle_max =  scan2_simulator.get_field_of_view()/2.;
        scan2_msg.angle_increment = scan2_simulator.get_angle_increment();
        scan2_msg.range_max = 100;
        scan2_msg.ranges = scan2_;
        scan2_msg.intensities = scan2_;

        scan2_pub.publish(scan2_msg);

        // Publish a transformation between base link and laser
        geometry_msgs::TransformStamped scan2_ts;
        scan2_ts.transform.translation.x = scan2_distance_to_base_link;
        scan2_ts.transform.rotation.z = 1;
        scan2_ts.transform.rotation.w = 0;
        scan2_ts.header.stamp = timestamp;
        scan2_ts.header.frame_id = base_frame;
        scan2_ts.child_frame_id = scan2_frame;
        br.sendTransform(scan2_ts);

      }
    }

    void pose_callback(const geometry_msgs::Pose & msg) {
      pose.x = msg.position.x;
      pose.y = msg.position.y;
      geometry_msgs::Quaternion q = msg.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      pose.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
      pose_callback(msg -> pose.pose);
    }

    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
      set_speed(msg.drive.speed);
      set_steering_angle(msg.drive.steering_angle, ros::Time::now());
    }

    void joy_callback(const sensor_msgs::Joy & msg) {
      set_speed(
          joy_max_speed * msg.axes[joy_speed_axis]);
      set_steering_angle(
          max_steering_angle * msg.axes[joy_angle_axis],
          ros::Time::now());
    }

    void set_speed(double speed_) {
      speed = std::min(std::max(speed_, -max_speed), max_speed);
    }

    void set_steering_angle(double steering_angle_, ros::Time timestamp) {
      steering_angle = std::min(std::max(steering_angle_, -max_steering_angle), max_steering_angle);

      // Publish the steering angle
      tf2::Quaternion quat;
      quat.setEuler(0., 0., steering_angle);
      geometry_msgs::TransformStamped ts;
      ts.transform.rotation.x = quat.x();
      ts.transform.rotation.y = quat.y();
      ts.transform.rotation.z = quat.z();
      ts.transform.rotation.w = quat.w();
      ts.header.stamp = timestamp;
      ts.header.frame_id = "front_left_hinge";
      ts.child_frame_id = "front_left_wheel";
      br.sendTransform(ts);
      ts.header.frame_id = "front_right_hinge";
      ts.child_frame_id = "front_right_wheel";
      br.sendTransform(ts);
    }

    void publish_obstacle(double x, double y, double radius){
      Obstacle _obj = Obstacle(x, y, radius);

      std::vector<Point2D> contour_vector;
      _obj.get_contour(contour_vector);

      geometry_msgs::PolygonStamped obs_msg;

      for (int i=0; i<contour_vector.size(); i++){
        geometry_msgs::Point32 pt;
        pt.x = (float)contour_vector[i].x;
        pt.y = (float)contour_vector[i].y;
        obs_msg.polygon.points.push_back(pt);
      }

      obs_msg.header.stamp = ros::Time::now();
      obs_msg.header.frame_id = "map";
      obstacle_pub.publish(obs_msg);
    }

    void obstacle_callback(const geometry_msgs::PointStamped & msg) {
      obstacle_simulator.add_static_object((double)msg.point.x, (double)msg.point.y);
      publish_obstacle((double)msg.point.x, (double)msg.point.y, obstacle_radius);
    }

    void dynamic_obstacle_callback(const geometry_msgs::PoseStamped & msg) {
      Pose2D _pose;
      _pose.x = msg.pose.position.x;
      _pose.y = msg.pose.position.y;
      geometry_msgs::Quaternion q = msg.pose.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      _pose.theta = tf2::impl::getYaw(quat);

      obstacle_simulator.add_dynamic_object((double)_pose.x, (double)_pose.y, _pose.theta, dynamic_obstacle_mode, ros::Time::now().toSec());
      publish_obstacle((double)_pose.x, (double)_pose.y, obstacle_radius);
    }

    void map_callback(const nav_msgs::OccupancyGrid & msg) {
      // Fetch the map parameters
      size_t height = msg.info.height;
      size_t width = msg.info.width;
      double resolution = msg.info.resolution;
      // Convert the ROS origin to a pose
      Pose2D origin;
      origin.x = msg.info.origin.position.x;
      origin.y = msg.info.origin.position.y;
      geometry_msgs::Quaternion q = msg.info.origin.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      origin.theta = tf2::impl::getYaw(quat);

      // Convert the map to probability values
      std::vector<double> map(msg.data.size());
      for (size_t i = 0; i < height * width; i++) {
        if (msg.data[i] > 100 or msg.data[i] < 0) {
          map[i] = 0.5; // Unknown
        } else {
          map[i] = msg.data[i]/100.;
        }
      }

      // Send the map to the scanner
      scan_simulator.set_map(
          map,
          height,
          width,
          resolution,
          origin,
          map_free_threshold);
      scan2_simulator.set_map(
          map,
          height,
          width,
          resolution,
          origin,
          map_free_threshold);

      obstacle_simulator = ObstacleSimulator(
        obstacle_radius, 
        obstacle_moving_speed,
        origin, 
        width, 
        height, 
        resolution
        );

      map_exists = true;
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "lightweight_robot_simulator");
  RobotSimulator rs;
  ros::spin();
  return 0;
}
