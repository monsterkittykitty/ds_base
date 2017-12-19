#ifndef DS_UDP_H
#define DS_UDP_H

#include "ds_base/ds_connection.h"
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include "ds_core_msgs/RawData.h"

using boost::asio::ip::udp;

class DsUdp : public DsConnection
{
public:
  DsUdp(boost::asio::io_service& io_service, std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle* myNh);

  virtual void receive(void);

  virtual void send(boost::shared_ptr<std::string> message);

  void setup(void);

  udp::socket& get_io_object(void);

private:

  void handle_receive(const boost::system::error_code& error,
		      std::size_t bytes_transferred);

  void handle_send(boost::shared_ptr<std::string> message,
		   const boost::system::error_code& error,
		   std::size_t bytes_transferred);

  boost::asio::io_service& io_service_;
  udp::socket* socket_;
  udp::endpoint* remote_endpoint_;
  boost::array<char, 128> recv_buffer_;
  boost::function<void(ds_core_msgs::RawData)> callback_;
  ros::NodeHandle* nh_;
  ros::Publisher raw_publisher_;
  ds_core_msgs::RawData raw_data_;
  std::string name_;
};

#endif
