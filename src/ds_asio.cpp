#include "ds_base/ds_asio.h"
#include "ds_base/ds_callbackqueue.h"

DsConnection* DsAsio::addConnection(boost::function<void(std::vector<unsigned char>)> callback)
{
  connections.push_back(new DsUdp(io_service, callback));
  return connections[connections.size() - 1];
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

  ROS_INFO_STREAM(ros::this_node::getName());
  ROS_INFO_STREAM(ros::this_node::getNamespace());
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

