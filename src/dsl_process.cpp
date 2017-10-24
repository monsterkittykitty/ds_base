#include <dsl_process.h>

DslProcess::DslProcess():
  spinner(4),
  socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 55555))
{
  ROS_INFO_STREAM("Hello, world!");
  spinner.start();
  io_service.run();
}

DslProcess::~DslProcess()
{
  ;
}
