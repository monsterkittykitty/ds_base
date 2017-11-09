#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
//#include <ds_connection.h>
#include <ds_udp.h>
#include <functional>

class DsAsio
{
public:
  DsAsio();
  ~DsAsio();

  template<typename T>
    void addRosSubscription(std::string channel, int queue, void (*callback)(T&));
  void addRosTimer(ros::Duration interval, void (*callback)(const ros::TimerEvent&));

  void addConnection(boost::function<void()> callback);
  
private:
  boost::asio::io_service io_service;
  ros::NodeHandle   nh;

  std::vector<DsConnection*> connections;
  
};
