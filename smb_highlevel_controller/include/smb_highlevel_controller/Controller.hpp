#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

namespace controller {

/**
 * Class containing the Controller
 */
class Controller {
 public:
  /** Constructor */
  Controller(ros::NodeHandle& nodeHandle);

  /** Destructor */
  virtual ~Controller();

 private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  bool controllArm(float pose[4]);
  bool controllGripper(bool openClose);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent& e);
  bool goForward(float xy_goal, bool xy);
  bool turn(double direction, bool leftRight);
  bool goForward(double xy_goal, bool xy);
  bool aproachObjectMinDistance(float front_dir_sen_length, float goal_distance);
  bool allignWithObject(int front_direction_index, int range_min_index);
  double x, y, yaw;
  bool rotating = false;
  bool moving_arm = false;
  int stage = 168;
  

  ros::NodeHandle nodeHandle_;
  ros::Publisher cmd_vel_publisher_;
  ros::Publisher vis_pub_;
  ros::Publisher arm_pub;
  ros::Publisher gripper_pub;
  ros::Subscriber odomSubscriber_;
  ros::Subscriber scanSubscriber_;
  ros::Timer timer_;
  geometry_msgs::Twist cmd_val_values_;
};

}  // namespace smb_highlevel_controller
