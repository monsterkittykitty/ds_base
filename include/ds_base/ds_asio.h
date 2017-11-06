#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

class DsAsio
{
public:
  DsAsio();
  ~DsAsio();

private:
  boost::asio::io_service io_service;
  ros::NodeHandle   nh;

};
