#include "ds_base/ds_connection_factory.h"

boost::shared_ptr<DsConnection> DsConnectionFactory::createConnection(ConnectionType connectionType, boost::asio::io_service& io_service, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle* myNh)
{
  switch (connectionType)
    {
    case UDP: return boost::shared_ptr<DsUdp>(new DsUdp(io_service, callback, myNh));
    case SERIAL: return boost::shared_ptr<DsSerial>(new DsSerial(io_service, callback, myNh));
    default:
      ROS_INFO_STREAM("Invalid connection type");
    }
}
