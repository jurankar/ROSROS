#include <ros/ros.h>
#include "smb_highlevel_controller/Controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle nodeHandle("~");

  controller::Controller Controller(nodeHandle);
  ros::AsyncSpinner spinner(4);

  spinner.start();
  ros::waitForShutdown();
  return 0;
}
