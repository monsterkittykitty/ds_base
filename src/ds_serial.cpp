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
  std::string port_name;
  nh_->param<std::string>(nh_->resolveName("port"), port_name, "/dev/ttyUSB0");
  ROS_INFO_STREAM("Serial port: " << port_name);

  int baud_rate;
  nh_->param<int>(nh_->resolveName("baud"), baud_rate, 9600);
  ROS_INFO_STREAM("Baud rate: " << port_name);

  char defaultc = '\n';
  eol_ = defaultc;
  //nh_->param<char>(nh_->resolveName("eol"), eol_, defaultc);
  ROS_INFO_STREAM("Eol character: " << eol_);
  
  port_ = new boost::asio::serial_port(io_service_, port_name);

  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  // The /raw channel should be appended to the nodehandle namespace
  raw_publisher_ = nh_->advertise<ds_core_msgs::RawData>("raw",1);
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
      for (unsigned int i = 0; i < bytes_transferred; ++i)
	{
	  char c = recv_buffer_[i];
	  ROS_INFO_STREAM("rxd: " << c);
	  if (c == eol_)
	    {	  
	      // Store timestamp as soon as received
	      raw_data_.header.io_time = ros::Time::now();
	      
	      ROS_INFO_STREAM("Serial received: " << raw_data_.data.data());
	      raw_data_.data_direction = ds_core_msgs::RawData::DATA_IN;
	      raw_publisher_.publish(raw_data_);
	      callback_(raw_data_);
	      raw_data_.data.clear();
	    }
	  else
	    {
	      ROS_INFO_STREAM("append: " << c);
	      raw_data_.data.push_back(c);
	    }
	}
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
  // Store timestamp as soon as received
  raw_data_.header.io_time = ros::Time::now();

  ROS_INFO_STREAM("Serial data sent");
  raw_data_.data = std::vector<unsigned char>(message->begin(), message->begin() + bytes_transferred);
  raw_data_.data_direction = ds_core_msgs::RawData::DATA_OUT;
  raw_publisher_.publish(raw_data_);
}

boost::asio::serial_port& DsSerial::get_io_object(void)
{
  return *port_;
}
