#ifndef DS_CONNECTION_H
#define DS_CONNECTION_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

class DsConnection
{
public:
  DsConnection();
  ~DsConnection();

  // Factory method
  static DsConnection* create(int type);

  virtual void send(boost::shared_ptr<std::string> message) = 0;
  
  virtual void receive(void) = 0;
};

#endif
