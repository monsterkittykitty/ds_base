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

class DsSerial : public DsConnection
{
public:
  DsSerial(boost::asio::io_service& io_service, boost::function<void(std::vector<unsigned char>)> callback, ros::NodeHandle* myNh);

  virtual void receive(void);

  virtual void send(boost::shared_ptr<std::string> message);

  void setup(void);
  
private:

  void handle_read(const boost::system::error_code& error,
		   std::size_t /*bytes_transferred*/);

  boost::asio::io_service& io_service_;
  boost::asio::serial_port* port_;
  boost::array<char, 1> recv_buffer_;
  boost::function<void(std::vector<unsigned char>)> callback_;
  ros::NodeHandle* nh_;
};


#endif
