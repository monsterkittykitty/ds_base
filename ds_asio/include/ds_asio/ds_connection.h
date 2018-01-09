#ifndef DS_CONNECTION_H
#define DS_CONNECTION_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include "ds_asio/ds_match_functions.h"

class DsConnection
{
public:
  DsConnection();
  ~DsConnection();

  /// @brief An interface to send data through a connection
  ///
  /// @param[in] message The message to send through the connection
  virtual void send(boost::shared_ptr<std::string> message) = 0;

  /// @brief An interface to receive data through a connection
  virtual void receive(void) = 0;
};

#endif
