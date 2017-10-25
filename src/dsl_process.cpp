#include <dsl_process.h>
#include "ros/callback_queue.h"

DslProcess::DslProcess():
  spinner(4),
  socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 55555))
{
  ros::CallbackQueue* queue = ros::getGlobalCallbackQueue();
  
  ros::WallDuration timeout(0.1f);
  while (nh.ok())
    {
      //queue->callAvailable(timeout);
      queue->callAvailable();
    }
  ROS_INFO_STREAM("Hello, world!");
  spinner.start();
  io_service.run();
}

DslProcess::~DslProcess()
{
  ;
}
