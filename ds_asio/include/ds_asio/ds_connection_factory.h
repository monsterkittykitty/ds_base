#ifndef DS_CONNECTION_FACTORY_H
#define DS_CONNECTION_FACTORY_H

#include "ds_asio/ds_connection.h"
#include "ds_asio/ds_udp.h"
#include "ds_asio/ds_serial.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>

namespace ds_asio
{

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

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  static boost::shared_ptr<DsConnection> createConnection(std::string name, boost::asio::io_service& io_service, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle& myNh, boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction)
  {
    std::string connectionType;
    myNh.getParam(ros::this_node::getName() + "/" + name + "/type", connectionType);

    if (connectionType.compare("UDP") == 0)
      return boost::shared_ptr<DsUdp>(new DsUdp(io_service, name, callback, &myNh));
    else if (connectionType.compare("SERIAL") == 0)
      return boost::shared_ptr<DsSerial>(new DsSerial(io_service, name, callback, &myNh));
  }
};

}


#endif
