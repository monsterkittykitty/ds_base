#ifndef DS_CONNECTION_H
#define DS_CONNECTION_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include "ds_asio/ds_match_functions.h"

#include <ds_core_msgs/RawData.h>

#include <ros/ros.h>

namespace ds_asio
{
/// @typedef The typedef for read callbacks
typedef boost::function<void(ds_core_msgs::RawData)> ReadCallback;
// typedef boost::function <void(ds_core_msgs::RawDataConstPtr)> ReadCallback;

/// \class The base class for all connection types.
///
/// Connections mostly talk to I/O devices like serial ports or UDP ports or whatever.
class DsConnection
{
public:
  DsConnection(boost::asio::io_service& _io);
  DsConnection(boost::asio::io_service& _io, std::string name, const ReadCallback& callback, ros::NodeHandle* myNh);
  virtual ~DsConnection();

  /// @brief An interface to send data through a connection
  ///
  /// @param[in] message The message to send through the connection
  virtual void send(boost::shared_ptr<std::string> message) = 0;

  /// @brief An overload to send a pure string
  ///
  /// @param[in] message The string that will get passed directly to the output character device.  Note that the
  /// string will be copied before it is sent out.
  virtual void send(const std::string& message);

  /// @brief Start async read loop
  virtual void receive(void) = 0;

  /// @brief Get this connection's name
  const std::string& getName() const;

  /// @brief Get the boost::asio service object underlying this one
  boost::asio::io_service& getIoService();

  /// @brief Get the callback
  const ReadCallback& getCallback() const;

  /// @brief Set the callback for this connection that is fired on every read
  void setCallback(const ReadCallback& _cb);

  // Make noncopyable
private:
  DsConnection(const DsConnection& other) = delete;             // non-copyconstructable
  DsConnection& operator=(const DsConnection& other) = delete;  // non-assignable

protected:
  ReadCallback callback_;
  boost::asio::io_service& io_service_;

  std::string name_;

  ros::NodeHandle* nh_;
  ros::Publisher raw_publisher_;
  ds_core_msgs::RawData raw_data_;
};
}
#endif
