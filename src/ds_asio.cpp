#include "ds_base/ds_asio.h"
#include "ds_base/ds_callbackqueue.h"

boost::shared_ptr<DsConnection> DsAsio::addConnection(std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::DsNodeHandle& myNh)
{
  connections.push_back(DsConnectionFactory::createConnection(name, io_service, callback, myNh));
  return connections[connections.size() - 1];
}

std::map<std::string, boost::shared_ptr<DsConnection> > DsAsio::startConnections(ros::DsNodeHandle& myNh, std::map<std::string, boost::function<void(ds_core_msgs::RawData data)> > mapping)
{
  std::map<std::string, boost::shared_ptr<DsConnection> > handle;

  // Iterate over mapping keys
  for ( const auto &myPair : mapping )
    {
      ROS_INFO_STREAM("Looking for connection: " << myPair.first);
      handle[myPair.first] = this->addConnection(myPair.first, mapping[myPair.first], myNh);
    }

  return handle;
}

DsAsio* DsAsio::asio(void)
{
  return this;
}

void DsAsio::addSub(std::string name, ros::Subscriber mySub)
{
  subs[name] = mySub;
}

void DsAsio::addTmr(std::string name, ros::Timer myTmr)
{
  tmrs[name] = myTmr;
}

void DsAsio::addPub(std::string name, ros::Publisher myPub)
{
  pubs[name] = myPub;
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

DsAsio::DsAsio() = default;

DsAsio::DsAsio(int argc, char** argv, const std::string &name)
  : DsAsio()
{
  ros::init(argc, argv, name);

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

