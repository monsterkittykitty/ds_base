#include "ds_base/ds_udp.h"

DsUdp::DsUdp(boost::asio::io_service& io_service, boost::function<void(std::vector<unsigned char>)> callback, ros::NodeHandle* myNh)
  : io_service_(io_service),
    DsConnection(),
    callback_(callback),
    nh_(myNh)
{
  setup();
  receive();
}

void DsUdp::setup(void)
{
  
  // This method, through nh, resolves params relative to nh namespace
  if (nh_->hasParam("rosdistro"))
    {
      ROS_INFO_STREAM("rosdistro exists");
      std::string rosdistro;
      nh_->getParam("rosdistro", rosdistro);
      ROS_INFO_STREAM(rosdistro);
    }
  else
    ROS_INFO_STREAM("rosdistro does not exist");

  if (nh_->hasParam("udp_rx"))
    {
      ROS_INFO_STREAM("udp_rx exists");
      int udp_rx;
      nh_->getParam("udp_rx", udp_rx);
      ROS_INFO_STREAM(udp_rx);
      socket_ = new udp::socket(io_service_, udp::endpoint(udp::v4(), udp_rx));
    }
  else
    {
      ROS_INFO_STREAM("udp_rx does not exist, default to port 44444");
      socket_ = new udp::socket(io_service_, udp::endpoint(udp::v4(), 44444));
    }
}

void DsUdp::receive(void)
{
  recv_buffer_.assign(0);
  //socket_.async_receive_from(boost::asio::buffer(recv_buffer_), remote_endpoint_,
  //			     boost::bind(&DsUdp::handle_receive, this,
  //					 boost::asio::placeholders::error,
  //					 boost::asio::placeholders::bytes_transferred));
  socket_->async_receive(boost::asio::buffer(recv_buffer_), 0,
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
  socket_->async_send_to(boost::asio::buffer(*message), myLocalHost,//remote_endpoint_,
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

