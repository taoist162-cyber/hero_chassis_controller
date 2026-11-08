#include <ros/ros.h>
#include "hero_chassis_controller/RosPackageTemplate.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hero_chassis_controller");
  ros::NodeHandle nodeHandle("~");

  hero_chassis_controller::RosPackageTemplate rosPackageTemplate(nodeHandle);

  ros::spin();
  return 0;
}
