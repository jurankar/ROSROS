#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

namespace smb_highlevel_controller {

/**
 * Class containing the SMB Highlevel Controller
 */
class SmbHighlevelController {
 public:
  /** Constructor */
  SmbHighlevelController(ros::NodeHandle& nodeHandle);

  /** Destructor */
  virtual ~SmbHighlevelController();

 private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void turn(int direction);

  ros::NodeHandle nodeHandle_;
  ros::Subscriber scanSubscriber_;
  ros::Publisher cmd_vel_publisher_;
  ros::Publisher vis_pub_;
};

}  // namespace smb_highlevel_controller
