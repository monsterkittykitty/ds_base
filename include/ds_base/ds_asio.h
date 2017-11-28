#ifndef DS_ASIO_H
#define DS_ASIO_H

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
  DsAsio(int argc, char** argv, const std::string &name);
  ~DsAsio();

  void addRosAdvertise(void);
  void addRosSubscription(std::string channel, int queue, boost::function<void(void)> callback);
  void addRosTimer(ros::Duration interval);

  void addConnection(boost::function<void(void)> callback);

  ros::NodeHandle& getNh(void);
  DsAsio* asio(void);

 private:
  boost::asio::io_service        io_service;
  ros::NodeHandle                *nh;

  std::vector<DsConnection*>     connections;
  std::vector<ros::Subscriber>   subs;
  std::vector<ros::Timer>        tmrs;
  std::vector<ros::Publisher>    pubs;
  
};

#endif
