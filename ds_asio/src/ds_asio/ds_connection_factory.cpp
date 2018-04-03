#include "ds_asio/ds_connection_factory.h"

namespace ds_asio
{
boost::shared_ptr<DsConnection>
DsConnectionFactory::createConnection(std::string name, boost::asio::io_service& io_service,
                                      boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle& myNh)
{
  std::string connectionType;
  myNh.getParam(ros::this_node::getName() + "/" + name + "/type", connectionType);

  if (connectionType.compare("UDP") == 0) {
    return boost::shared_ptr<DsUdp>(new DsUdp(io_service, name, callback, myNh));
  } else if (connectionType.compare("SERIAL") == 0) {
    return boost::shared_ptr<DsSerial>(new DsSerial(io_service, name, callback, myNh));
  }

  std::stringstream msg;
  msg <<"Unable to create connection \"" <<name <<"\" in node " <<ros::this_node::getName()
      <<" with type \"" <<connectionType <<"\"!\nLooking for type rosparam at: \""
      <<ros::this_node::getName() + "/" + name + "/type\"";

  throw std::runtime_error(msg.str());
}
}
