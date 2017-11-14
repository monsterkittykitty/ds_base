#include <ds_asio.h>
#include <ds_callbackqueue.h>
#include "std_msgs/String.h"


void testCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Callback!");
}

void timerCallback(const ros::TimerEvent& msg)
{
  ROS_INFO_STREAM("Timer callback!");
}

void connCallback(void)
{
  ROS_INFO_STREAM("Conn callback!");
}

void DsAsio::addRosAdvertise(void)
{
  ros::AdvertiseOptions ops;
  pubs.push_back(nh.advertise(ops));
}

void DsAsio::addRosSubscription(std::string channel, int queue, boost::function<void(void)> callback)
{
  subs.push_back(nh.subscribe<std_msgs::String>(channel, queue, boost::bind(testCallback, _1)));
}

void DsAsio::addRosTimer(ros::Duration interval)
{
  tmrs.push_back(nh.createTimer(interval, boost::bind(timerCallback, _1)));
}

void DsAsio::addConnection(boost::function<void(void)> callback)
{
  connections.push_back(new DsUdp(io_service, callback));
  connections[connections.size()]->receive(callback);
}

DsAsio::DsAsio()
{
  ros::DsCallbackQueue* queue = new ros::DsCallbackQueue(&io_service);
  nh.setCallbackQueue((ros::CallbackQueue*) queue);

  //addConnection(boost::bind(connCallback));

  addRosSubscription("test",1000,connCallback);
  addRosTimer(ros::Duration(0.5));
  
  // Work object prevents io_service from quitting while it exists
  boost::asio::io_service::work work(io_service);
  io_service.run();
}

DsAsio::~DsAsio()
{
}

