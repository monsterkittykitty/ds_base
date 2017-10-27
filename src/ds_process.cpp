#include <ds_process.h>
#include <ds_callbackqueue.h>
#include "ros/callback_queue.h"
#include "std_msgs/String.h"

void testCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Callback!");
}

void testCallback2(void)
{
  ROS_INFO_STREAM("Callback2!");
}

DsProcess::DsProcess():
  spinner(4),
  socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 55555))
{
  ros::DsCallbackQueue* queue = new ros::DsCallbackQueue(&io_service);
  //queue->registerIoService(&io_service);
  nh.setCallbackQueue((ros::CallbackQueue*) queue);

  //ros::CallbackQueue* queue = new ros::CallbackQueue();
  //nh.setCallbackQueue((ros::CallbackQueue*) queue);
  
  ROS_INFO_STREAM("Hello, world!");

  // Work object prevents io_service from quitting while it exists
  boost::asio::io_service::work work(io_service);
  
  ros::Subscriber sub = nh.subscribe("test", 1000, &testCallback);
  //ros::AsyncSpinner spinner(0, queue);
  //spinner.start();
  io_service.post(boost::bind(&testCallback2));
  ROS_INFO_STREAM("Running io_service!");
  io_service.run();
  ROS_INFO_STREAM("Stopped running io_service!");

}

DsProcess::~DsProcess()
{
  ;
}
