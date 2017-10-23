#include <ros/ros.h>
#include <dsl_process.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spinner_node");
  DslProcess myProcess;
  ros::waitForShutdown();
  return 0;
}
