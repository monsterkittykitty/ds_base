#ifndef DS_UDP_H
#define DS_UDP_H

#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <ds_connection.h>

using boost::asio::ip::udp;

class DsUdp : public DsConnection
{
public:
  DsUdp(boost::asio::io_service& io_service, boost::function<void(std::string)> callback);

  virtual void receive(void);

  virtual void send(boost::shared_ptr<std::string> /*message*/,
		    const boost::system::error_code& /*error*/,
		    std::size_t /*bytes_transferred*/);

private:

  void handle_receive(const boost::system::error_code& error,
		      std::size_t /*bytes_transferred*/);

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, 128> recv_buffer_;
  boost::function<void(std::string)> callback_;
};

#endif
