#include <dsl_process.h>

DslProcess::DslProcess():
  spinner(4)
{
  ROS_INFO_STREAM("Hello, world!");
  spinner.start();
}

DslProcess::~DslProcess()
{
  ;
}
