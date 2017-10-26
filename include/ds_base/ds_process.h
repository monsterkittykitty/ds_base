#include <ros/ros.h>
#include <boost/asio.hpp>

class DsProcess
{
public:
  DsProcess();
  ~DsProcess();

protected:
  ros::NodeHandle   nh;
  ros::AsyncSpinner spinner;

private:
  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket socket_;
};
