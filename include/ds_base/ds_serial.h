#ifndef DS_SERIAL_H
#define DS_SERIAL_H

#include "ds_base/ds_connection.h"
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "ds_core_msgs/RawData.h"

class DsSerial : public DsConnection
{
public:
  DsSerial(boost::asio::io_service& io_service, std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle* myNh);

  virtual void receive(void);

  virtual void send(boost::shared_ptr<std::string> message);

  void setup(void);

  boost::asio::serial_port& get_io_object(void);

private:

  void handle_read(const boost::system::error_code& error,
		   std::size_t bytes_transferred);

  void handle_write(boost::shared_ptr<std::string> message,
		   const boost::system::error_code& error,
		   std::size_t bytes_transferred);

  boost::asio::io_service& io_service_;
  boost::asio::serial_port* port_;
  boost::array<char, 1> recv_buffer_;
  boost::function<void(ds_core_msgs::RawData)> callback_;
  ros::NodeHandle* nh_;
  ros::Publisher raw_publisher_;
  ds_core_msgs::RawData raw_data_;
  char eol_;
};


#endif
