#include "ds_base/ds_udp.h"

DsUdp::DsUdp(boost::asio::io_service& io_service, boost::function<void(std::vector<unsigned char>)> callback)
  : socket_(io_service, udp::endpoint(udp::v4(), 44444)),
    DsConnection(),
    callback_(callback)
{
  receive();
}

void DsUdp::receive(void)
{
  recv_buffer_.assign(0);
  //socket_.async_receive_from(boost::asio::buffer(recv_buffer_), remote_endpoint_,
  //			     boost::bind(&DsUdp::handle_receive, this,
  //					 boost::asio::placeholders::error,
  //					 boost::asio::placeholders::bytes_transferred));
  socket_.async_receive(boost::asio::buffer(recv_buffer_), 0,
			boost::bind(&DsUdp::handle_receive, this,
				    boost::asio::placeholders::error,
				    boost::asio::placeholders::bytes_transferred));

}

void DsUdp::handle_receive(const boost::system::error_code& error,
			   std::size_t /*bytes_transferred*/)
{
  if (!error || error == boost::asio::error::message_size)
    {
      ROS_INFO_STREAM("UDP received: " << recv_buffer_.data());
      std::vector<unsigned char> data(recv_buffer_.begin(), recv_buffer_.end());
      callback_(data);
      receive();
    }
}

void DsUdp::send(boost::shared_ptr<std::string> message)
{
  ROS_INFO_STREAM("Scheduling UDP send");
  udp::endpoint myLocalHost(boost::asio::ip::address::from_string("127.0.0.1"), 44443);
  socket_.async_send_to(boost::asio::buffer(*message), myLocalHost,//remote_endpoint_,
			boost::bind(&DsUdp::handle_send, this, message,
				    boost::asio::placeholders::error,
				    boost::asio::placeholders::bytes_transferred));
  ROS_INFO_STREAM("UDP send scheduled");
}

void DsUdp::handle_send(boost::shared_ptr<std::string> message,
			const boost::system::error_code& error,
			std::size_t bytes_transferred)
{
  ROS_INFO_STREAM("UDP data sent");
}

