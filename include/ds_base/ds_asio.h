#ifndef DS_ASIO_H
#define DS_ASIO_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include "ds_base/ds_udp.h"

class DsAsio
{
public:
  DsAsio(int argc, char** argv, const std::string &name);
  ~DsAsio();

  void run(void);

  DsConnection* addConnection(boost::function<void(std::string)> callback);

  ros::NodeHandle& getNh(void);
  DsAsio* asio(void);

  void addSub(ros::Subscriber mySub);
  void addTmr(ros::Timer myTmr);
  void addPub(ros::Publisher myPub);
  
 private:
  boost::asio::io_service        io_service;
  ros::NodeHandle                *nh;

  std::vector<DsConnection*>     connections;
  std::vector<ros::Subscriber>   subs;
  std::vector<ros::Timer>        tmrs;
  std::vector<ros::Publisher>    pubs;
  
};

#endif
