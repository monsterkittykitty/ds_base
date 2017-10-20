#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spinner_node");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Hello, world!");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
