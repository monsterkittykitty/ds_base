#include <ds_asio.h>
#include <ds_callbackqueue.h>
#include "std_msgs/String.h"


void testCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Callback!");
}

void timerCallback(const ros::TimerEvent&)
{
  ROS_INFO_STREAM("Timer callback!");
}

template<typename T>
void DsAsio::addRosSubscription(std::string channel, int queue, void (*callback)(T&))
{
  ros::Subscriber sub = nh.subscribe(channel, queue, callback);
}

void DsAsio::addRosTimer(ros::Duration interval, void (*callback)(const ros::TimerEvent&))
{
  ros::Timer timer = nh.createTimer(interval, callback);
}

// SS - add a callback for the data
void DsAsio::addConnection(void)
{
  connections.push_back(new DsUdp(io_service));
  connections[connections.size()]->receive();
}

DsAsio::DsAsio()
{
  ros::DsCallbackQueue* queue = new ros::DsCallbackQueue(&io_service);
  nh.setCallbackQueue((ros::CallbackQueue*) queue);

  addRosTimer(ros::Duration(0.5), timerCallback);
  ros::Subscriber sub = nh.subscribe("test", 1000, &testCallback);
  
  // Work object prevents io_service from quitting while it exists
  boost::asio::io_service::work work(io_service);
  io_service.run();
}

DsAsio::~DsAsio()
{
}

