#include <ds_udp.h>

DsUdp::DsUdp(boost::asio::io_service& io_service, boost::function<void()> callback)
  : socket_(io_service, udp::endpoint(udp::v4(), 44444)),
    DsConnection(),
    callback_(callback)
{
  receive(callback_);
}

void DsUdp::receive(boost::function<void()> callback)
{
  socket_.async_receive_from(boost::asio::buffer(recv_buffer_), remote_endpoint_,
			     boost::bind(&DsUdp::handle_receive, this,
					 boost::asio::placeholders::error,
					 boost::asio::placeholders::bytes_transferred));

  callback();
}

void DsUdp::handle_receive(const boost::system::error_code& error,
			   std::size_t /*bytes_transferred*/)
{
  if (!error || error == boost::asio::error::message_size)
    {
      boost::shared_ptr<std::string> message(
					     new std::string("TEST"));

      socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
			    boost::bind(&DsUdp::send, this, message,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));

      receive(callback_);
    }
}

void DsUdp::send(boost::shared_ptr<std::string> /*message*/,
			 const boost::system::error_code& /*error*/,
			 std::size_t /*bytes_transferred*/)
{
  ROS_INFO_STREAM("Sending UDP reply");
}

