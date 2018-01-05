#include "ds_base/ds_serial.h"
#include <cstdlib>

DsSerial::DsSerial(boost::asio::io_service& io_service, std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::NodeHandle* myNh)
  : io_service_(io_service),
    DsConnection(),
    callback_(callback),
    nh_(myNh),
    name_(name)
{
  setup();
  receive();
}

void DsSerial::setup(void)
{
  std::string port_name;
  nh_->param<std::string>(ros::this_node::getName() + "/" + name_ + "/port", port_name, "/dev/ttyUSB0");
  ROS_INFO_STREAM("Serial port: " << port_name);

  int baud_rate;
  nh_->param<int>(ros::this_node::getName() + "/" + name_ + "/baud", baud_rate, 9600);
  ROS_INFO_STREAM("Baud rate: " << baud_rate);

  int data_bits;
  nh_->param<int>(ros::this_node::getName() + "/" + name_ + "/data_bits", data_bits, 8);
  ROS_INFO_STREAM("Data bits: " << data_bits);

  std::string myMatch;
  nh_->param<std::string>(ros::this_node::getName() + "/" + name_ + "/matcher", myMatch, "match_char");
  ROS_INFO_STREAM("Matcher: " << myMatch);
  if (!myMatch.compare("match_char"))
    {
      //char delimiter;
      std::string hexAscii;
      nh_->param<std::string>(ros::this_node::getName() + "/" + name_ + "/delimiter", hexAscii, "0A");
      unsigned int delimiter;
      sscanf(hexAscii.c_str(), "%X", &delimiter);
      ROS_INFO_STREAM("Hex ascii" << hexAscii << hexAscii.c_str());
      set_matcher(match_char((char) delimiter));
    }
  
  port_ = new boost::asio::serial_port(io_service_, port_name);

  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  port_->set_option(boost::asio::serial_port_base::character_size(data_bits));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  // The /raw channel should be appended to the nodehandle namespace
  raw_publisher_ = nh_->advertise<ds_core_msgs::RawData>(ros::this_node::getName() + "/" + name_ + "/raw",1);
}

void DsSerial::receive(void)
{
  //recv_buffer_.assign(0);
  // boost::asio::async_read(*port_, boost::asio::buffer(recv_buffer_),
  // 			  boost::bind(&DsSerial::handle_read, this,
  // 				      boost::asio::placeholders::error,
  // 				      boost::asio::placeholders::bytes_transferred));
  //boost::asio::streambuf b;
  // boost::asio::async_read_until(*port_, streambuf_, match_char('\n'),
  // 				boost::bind(&DsSerial::handle_read, this,
  // 					    boost::asio::placeholders::error,
  // 					    boost::asio::placeholders::bytes_transferred));
  boost::asio::async_read_until(*port_, streambuf_, matchFunction_,
				boost::bind(&DsSerial::handle_read, this,
					    boost::asio::placeholders::error,
					    boost::asio::placeholders::bytes_transferred));
  
}

void DsSerial::set_matcher(boost::function<std::pair<iterator, bool>(iterator, iterator)> matchFunction)
{
  matchFunction_ = matchFunction;
}

void DsSerial::handle_read(const boost::system::error_code& error,
			std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
    {
      // Store timestamp as soon as received
      raw_data_.ds_header.io_time = ros::Time::now();

      boost::asio::streambuf::const_buffers_type bufs = streambuf_.data();
      raw_data_.data = std::vector<unsigned char>(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + bytes_transferred);
      ROS_INFO_STREAM("Serial received: " << raw_data_.data.data());
      raw_data_.data_direction = ds_core_msgs::RawData::DATA_IN;
      raw_publisher_.publish(raw_data_);
      callback_(raw_data_);
      raw_data_.data.clear();
      // The consume method of the strembuffer marks as used the bytes that we last processed, so the next call to async_read_until does not re-analyze them
      streambuf_.consume(bytes_transferred);
      // for (unsigned int i = 0; i < bytes_transferred; ++i)
      // 	{
      // 	  char c = recv_buffer_[i];
      // 	  ROS_INFO_STREAM("rxd: " << c);
      // 	  if (c == eol_.at(0))
      // 	    {	  
      // 	      // Store timestamp as soon as received
      // 	      raw_data_.ds_header.io_time = ros::Time::now();
	      
      // 	      ROS_INFO_STREAM("Serial received: " << raw_data_.data.data());
      // 	      raw_data_.data_direction = ds_core_msgs::RawData::DATA_IN;
      // 	      raw_publisher_.publish(raw_data_);
      // 	      callback_(raw_data_);
      // 	      raw_data_.data.clear();
      // 	    }
      // 	  else
      // 	    {
      // 	      ROS_INFO_STREAM("append: " << c);
      // 	      raw_data_.data.push_back(c);
      // 	    }
      // 	}
      // receive(); // This should be out of the if(!error) condition, otherwise we stop receiving if there is an error condition
    }
  receive();
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
  raw_data_.ds_header.io_time = ros::Time::now();

  ROS_INFO_STREAM("Serial data sent");
  raw_data_.data = std::vector<unsigned char>(message->begin(), message->begin() + bytes_transferred);
  raw_data_.data_direction = ds_core_msgs::RawData::DATA_OUT;
  raw_publisher_.publish(raw_data_);
  raw_data_.data.clear();
}

boost::asio::serial_port& DsSerial::get_io_object(void)
{
  return *port_;
}
