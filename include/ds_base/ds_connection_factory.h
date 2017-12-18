#ifndef DS_CONNECTION_FACTORY_H
#define DS_CONNECTION_FACTORY_H

#include "ds_base/ds_connection.h"
#include "ds_base/ds_udp.h"
#include "ds_base/ds_serial.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>

class DsConnectionFactory
{
public:
  enum ConnectionType
  {
    UDP,
    SERIAL
  };

  static boost::shared_ptr<DsConnection> createConnection(std::string connectionType, std::string name, boost::asio::io_service& io_service, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle& myNh);

};



#endif
