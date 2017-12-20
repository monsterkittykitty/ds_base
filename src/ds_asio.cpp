#include "ds_base/ds_asio.h"
#include "ds_base/ds_callbackqueue.h"

boost::shared_ptr<DsConnection> DsAsio::addConnection(std::string type, std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::DsNodeHandle& myNh)
{
  connections.push_back(DsConnectionFactory::createConnection(type, name, io_service, callback, myNh));
  return connections[connections.size() - 1];
}

std::map<std::string, boost::shared_ptr<DsConnection> > DsAsio::startConnections(ros::DsNodeHandle& myNh, std::map<std::string, boost::function<void(ds_core_msgs::RawData data)> > mapping)
{
  std::map<std::string, boost::shared_ptr<DsConnection> > handle;

  // TODO! Check that the mapping matches what is in the parameter server
  XmlRpc::XmlRpcValue v;
  std::vector<std::string> connType;
  if (myNh.hasParam(ros::this_node::getName() + "/conn"))
    {
      ROS_INFO_STREAM("conn exists: " << ros::this_node::getName() + "/conn");
      ros::param::get(ros::this_node::getName() + "/conn", v);
      for(int i =0; i < v.size(); i++)
	{
	  connType.push_back(v[i]);
	  ROS_INFO_STREAM("connType: " << connType[i]);
	}
    }
  std::vector<std::string> connName;
  if (myNh.hasParam(ros::this_node::getName() + "/conn_name"))
    {
      ROS_INFO_STREAM("conn_name exists: " << ros::this_node::getName() + "/conn_name");
      ros::param::get(ros::this_node::getName() + "/conn_name", v);
      for(int i =0; i < v.size(); i++)
	{
	  connName.push_back(v[i]);
	  ROS_INFO_STREAM("connName: " << connName[i]);
	}
    }
  for (int i = 0; i < connType.size(); ++i)
    handle[connName[i]] = this->addConnection(connType[i], connName[i], mapping[connName[i]], myNh);

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

DsAsio::DsAsio(int argc, char** argv, const std::string &name)
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

