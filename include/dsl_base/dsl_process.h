#include <ros/ros.h>

class DslProcess
{
public:
  DslProcess();
  ~DslProcess();

protected:
  ros::NodeHandle   nh;
  ros::AsyncSpinner spinner;

};
