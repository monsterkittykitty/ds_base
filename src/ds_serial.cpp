#include "ds_base/ds_serial.h"

DsSerial::DsSerial(boost::asio::io_service& io_service, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle* myNh)
  : io_service_(io_service),
    DsConnection(),
    callback_(callback),
    nh_(myNh)
{
  setup();
  receive();
}

void DsSerial::setup(void)
{
  port_ = new boost::asio::serial_port(io_service_);  

  // The /raw channel should be appended to the nodehandle namespace
  raw_publisher_ = nh_->advertise<ds_core_msgs::RawData>("/raw",1);
}

void DsSerial::receive(void)
{
  recv_buffer_.assign(0);
  boost::asio::async_read(*port_, boost::asio::buffer(recv_buffer_),
			  boost::bind(&DsSerial::handle_read, this,
				      boost::asio::placeholders::error,
				      boost::asio::placeholders::bytes_transferred));

}

void DsSerial::handle_read(const boost::system::error_code& error,
			std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
    {
      // Store timestamp as soon as received
      raw_data_.header.io_time = ros::Time::now();

      // Need to add delimiter test, etc...
      ROS_INFO_STREAM("Serial received: " << recv_buffer_.data());
      raw_data_.data = std::vector<unsigned char>(recv_buffer_.begin(), recv_buffer_.begin() + bytes_transferred);
      raw_data_.data_direction = ds_core_msgs::RawData::DATA_IN;
      raw_publisher_.publish(raw_data_);
      callback_(raw_data_);
      receive();
    }
}

void DsSerial::send(boost::shared_ptr<std::string> message)
{
  ROS_INFO_STREAM("Scheduling serial send");
  boost::asio::async_write(*port_, boost::asio::buffer(*message),
			   boost::bind(&DsSerial::handle_write, this, message,
				       boost::asio::placeholders::error,
				       boost::asio::placeholders::bytes_transferred));

  ROS_INFO_STREAM("Serial send scheduled");
}

void DsSerial::handle_write(boost::shared_ptr<std::string> message,
			    const boost::system::error_code& error,
			    std::size_t bytes_transferred)
{
  ROS_INFO_STREAM("Serial data sent");
}

