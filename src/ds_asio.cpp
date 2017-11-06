#include <ds_asio.h>
#include <ds_udp.h>
#include <ds_callbackqueue.h>
#include "std_msgs/String.h"


void testCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Callback!");
}

void testCallback2(void)
{
  ROS_INFO_STREAM("Callback2!");
}

void timerCallback(const ros::TimerEvent&)
{
  ROS_INFO_STREAM("Timer callback!");
}


DsAsio::DsAsio()
{

  DsUdp server(io_service);
  server.receive();

  ros::DsCallbackQueue* queue = new ros::DsCallbackQueue(&io_service);
  nh.setCallbackQueue((ros::CallbackQueue*) queue);

  ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);
  
  ros::Subscriber sub = nh.subscribe("test", 1000, &testCallback);
  
  // Work object prevents io_service from quitting while it exists
  boost::asio::io_service::work work(io_service);
  io_service.run();
}

DsAsio::~DsAsio()
{
}

