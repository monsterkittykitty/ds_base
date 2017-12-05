#include "ds_base/ds_asio.h"
#include "ds_base/ds_callbackqueue.h"
//#include "std_msgs/String.h"

DsConnection* DsAsio::addConnection(boost::function<void(std::string)> callback)
{
  connections.push_back(new DsUdp(io_service, callback));
  //connections[connections.size()]->receive(callback);
  return connections[connections.size()];
}

ros::NodeHandle& DsAsio::getNh(void)
{
  return *nh;
}

DsAsio* DsAsio::asio(void)
{
  return this;
}

void DsAsio::addSub(ros::Subscriber mySub)
{
  subs.push_back(mySub);
}

void DsAsio::addTmr(ros::Timer myTmr)
{
  tmrs.push_back(myTmr);
}

void DsAsio::addPub(ros::Publisher myPub)
{
  pubs.push_back(myPub);
}

DsAsio::DsAsio(int argc, char** argv, const std::string &name)
{
  ros::init(argc, argv, name);

  nh = new ros::NodeHandle();
  ros::DsCallbackQueue* queue = new ros::DsCallbackQueue(&io_service);
  nh->setCallbackQueue((ros::CallbackQueue*) queue);

  //addConnection(boost::bind(connCallback));

  //addRosSubscription("test",1000,connCallback);
  //addRosTimer(ros::Duration(0.5));

  ROS_INFO_STREAM(ros::this_node::getName());
  ROS_INFO_STREAM(ros::this_node::getNamespace());
  
  // Work object prevents io_service from quitting while it exists
  //boost::asio::io_service::work work(io_service);
  //io_service.run();
}

void DsAsio::run(void)
{
  // Work object prevents io_service from quitting while it exists
  boost::asio::io_service::work work(io_service);
  io_service.run();
}

DsAsio::~DsAsio()
{
}

