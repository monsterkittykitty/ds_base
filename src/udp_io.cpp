#include <udp_io.h>

std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  ROS_INFO_STREAM("Generating time string");
  return ctime(&now);
}

udp_server::udp_server(boost::asio::io_service& io_service)
  : socket_(io_service, udp::endpoint(udp::v4(), 44444))
{
  start_receive();
}

void udp_server::start_receive()
{
  socket_.async_receive_from(
			     boost::asio::buffer(recv_buffer_), remote_endpoint_,
			     boost::bind(&udp_server::handle_receive, this,
					 boost::asio::placeholders::error,
					 boost::asio::placeholders::bytes_transferred));
}

void udp_server::handle_receive(const boost::system::error_code& error,
				std::size_t /*bytes_transferred*/)
{
  if (!error || error == boost::asio::error::message_size)
    {
      boost::shared_ptr<std::string> message(
					     new std::string(make_daytime_string()));

      socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
			    boost::bind(&udp_server::handle_send, this, message,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));

      start_receive();
    }
}

void udp_server::handle_send(boost::shared_ptr<std::string> /*message*/,
			     const boost::system::error_code& /*error*/,
			     std::size_t /*bytes_transferred*/)
{
  ROS_INFO_STREAM("Sending UDP reply");
}

