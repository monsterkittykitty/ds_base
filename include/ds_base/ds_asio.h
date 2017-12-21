#ifndef DS_ASIO_H
#define DS_ASIO_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include "ds_base/ds_udp.h"
#include "ds_base/ds_serial.h"
#include "ds_base/ds_connection_factory.h"
#include "ds_base/ds_nodehandle.h"
#include "ds_core_msgs/RawData.h"

class DsAsio
{
public:

  DsAsio();
  DsAsio(int argc, char** argv, const std::string &name);
  ~DsAsio();

  void run(void);

  boost::shared_ptr<DsConnection> addConnection(std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::DsNodeHandle& myNh);

  DsAsio* asio(void);

  void addSub(std::string name, ros::Subscriber mySub);
  void addTmr(std::string name, ros::Timer myTmr);
  void addPub(std::string name, ros::Publisher myPub);

  std::map<std::string, boost::shared_ptr<DsConnection> > startConnections(ros::DsNodeHandle& myNh, std::map<std::string, boost::function<void(ds_core_msgs::RawData data)> > mapping);

  void signalHandler(const boost::system::error_code& error, int signal_number);
  
  boost::asio::io_service                        io_service;

 protected:
  std::map<std::string, ros::Subscriber>         subs;
  std::map<std::string, ros::Timer>              tmrs;
  std::map<std::string, ros::Publisher>          pubs;
 
 private:

  std::vector<boost::shared_ptr<DsConnection> >  connections;
  
};

#endif
