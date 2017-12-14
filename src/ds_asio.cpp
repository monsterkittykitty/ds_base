#include "ds_base/ds_asio.h"
#include "ds_base/ds_callbackqueue.h"

DsConnection* DsAsio::addConnection(std::string type, std::string name, boost::function<void(ds_core_msgs::RawData)> callback)
{
  if (type.compare("UDP") == 0)
    {
      connections.push_back(new DsUdp(io_service, callback, this->getNhPtr()));
      return connections[connections.size() - 1];
    }
  else if (type.compare("SERIAL") == 0)
    {
      //connections.push_back(new DsSerial(io_service, callback, this->getNhPtr()));
      return connections[connections.size() - 1];
    }
  else
    {
      return NULL;
    }
}

ros::NodeHandle& DsAsio::getNh(void)
{
  return *nh;
}

ros::NodeHandle* DsAsio::getNhPtr(void)
{
  return nh;
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

void DsAsio::signalHandler(const boost::system::error_code& error, int signal_number)
{
  if (!error)
    {
      ROS_INFO_STREAM("A signal occurred, shutting down ROS and exiting...");
      ros::shutdown();
      exit(0);
    }
  else
    {
      ROS_INFO_STREAM("An error in the signal handler occurred");
    }
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
  boost::asio::signal_set signals(io_service, SIGINT);
  signals.async_wait(boost::bind(&DsAsio::signalHandler, this, _1, _2));
  io_service.run();
}

DsAsio::~DsAsio()
{
}

