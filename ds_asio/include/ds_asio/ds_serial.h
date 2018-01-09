#ifndef DS_SERIAL_H
#define DS_SERIAL_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>

namespace ds_asio
{

class DsSerial : public DsConnection
{
public:
  DsSerial(boost::asio::io_service& io_service, std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle* myNh);

  virtual void receive(void);

  virtual void send(boost::shared_ptr<std::string> message);

  void setup(void);

  boost::asio::serial_port& get_io_object(void);

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;
  void set_matcher(boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction);

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
  //char eol_;
  std::string eol_; // end of line, may be a single character
  std::string name_;
  boost::asio::streambuf streambuf_;
  boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction_;

};

}

#endif
