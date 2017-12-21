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
  /// @brief Generates a new connection object
  ///
  /// @return A boost::shared_ptr of the created connection 
  ///
  /// @param[in] name Name of the connection, defines subnamespace of connection parameters
  /// @param[in] io_service   A reference to the boost::io_service object that the connection will use to perform i/o
  /// @param[in] callback     A boost::function object that will be called with the data received when the connection receives data
  /// @param[in] myNh         A reference to the nodehandle object for accessing the parameter server
  static boost::shared_ptr<DsConnection> createConnection(std::string name, boost::asio::io_service& io_service, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle& myNh);
};



#endif
