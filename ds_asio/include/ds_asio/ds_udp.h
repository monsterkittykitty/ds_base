#ifndef DS_UDP_H
#define DS_UDP_H

#include "ds_asio/ds_connection.h"
#include "ds_core_msgs/RawData.h"

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <ctime>
#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>

using boost::asio::ip::udp;

namespace ds_asio
{
class DsUdp : public DsConnection
{
public:
  DsUdp(boost::asio::io_service& io_service, std::string name, const ReadCallback& callback, ros::NodeHandle& myNh);

  void receive(void) override;

  void send(boost::shared_ptr<std::string> message) override;

  void setup(ros::NodeHandle& nh) override;

  udp::socket& get_io_object(void);

private:
  // make noncopyable
  DsUdp(const DsUdp& other) = delete;
  DsUdp& operator=(const DsUdp& other) = delete;

private:
  void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);

  void handle_send(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                   std::size_t bytes_transferred);

  std::unique_ptr<udp::socket> socket_;
  udp::endpoint* remote_endpoint_;
  boost::array<char, 512> recv_buffer_;
  uint8_t num_read_error_;
  ros::Timer read_error_retry_timer_;
};
}
#endif
