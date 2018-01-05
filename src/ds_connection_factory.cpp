#include "ds_base/ds_connection_factory.h"

boost::shared_ptr<DsConnection> DsConnectionFactory::createConnection(std::string name, boost::asio::io_service& io_service, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle& myNh)
{
  std::string connectionType;
  myNh.getParam(ros::this_node::getName() + "/" + name + "/type", connectionType);

  if (connectionType.compare("UDP") == 0)
    return boost::shared_ptr<DsUdp>(new DsUdp(io_service, name, callback, &myNh));
  else if (connectionType.compare("SERIAL") == 0)
    return boost::shared_ptr<DsSerial>(new DsSerial(io_service, name, callback, &myNh));

}

