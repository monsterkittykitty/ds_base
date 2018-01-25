#ifndef DS_ASIO_H
#define DS_ASIO_H

#include "ds_asio/ds_udp.h"
#include "ds_asio/ds_serial.h"
#include "ds_asio/ds_connection_factory.h"
#include "ds_asio/ds_nodehandle.h"
#include "ds_asio/ds_iosm.h"
#include "ds_core_msgs/RawData.h"
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <unordered_map>

namespace ds_asio
{
class DsAsio
{
public:
  // See ds_connection.h for the ReadCallback typedef

  /// @brief Default constructor
  DsAsio();

  /// @brief Constructor that calls ros::init(argc, argv, name)
  ///
  /// @param[in] argc Number of input command line arguments
  /// @param[in] argv Command line arguments
  /// @param[in] name Name of the ros process. may be remapped by ros::init if using roslaunch
  DsAsio(int argc, char** argv, const std::string& name);

  /// @brief Destructor
  ~DsAsio();

  /// @brief Run the owned boost::io_service event loop.
  ///
  /// This method blocks until terminated by signals.
  void run(void);

  /// @brief Method to add a single connection. Calls factory class to generate connections
  ///
  /// @param[in] name Name of the connection. Will be used to index the ros parameter server namespace
  /// @param[in] callback A boost::function object that will be called with the data received when the connection
  /// receives data
  /// @param[in] myNh A reference to the nodehandle object for accessing the parameter server
  ///
  /// @return A boost::shared_ptr object that is a handle for the created connection
  boost::shared_ptr<DsConnection> addConnection(std::string name, const ReadCallback& callback, DsNodeHandle& myNh);

  /// @brief Method to add an IO state machine and its associated connection
  ///
  /// This function will also call addConnection to initialize the underlying connection class before
  /// connecting it to the I/O state machine.
  ///
  /// The connection can be accessed with:
  ///     boost::shared_ptr<IoSM> iosm = dsasio.addIoSM( <parameters> );
  ///     boost::shared_ptr<DsConnection> conn = iosm->getConnection();
  ///
  /// \param iosm_name Io state machine name.  Will be used to index the ros parameter server namespace
  /// \param conn_name The name for the underlying connection object.
  /// \param callback A boost::function object that will be called when the connection has received data
  /// \param myNh A reference to the node handle for accessing the parameter server
  ///
  /// \return A boost::shared_ptr object with the Io state machine
  boost::shared_ptr<ds_asio::IoSM> addIoSM(std::string iosm_name, std::string conn_name, const ReadCallback& callback,
                                           DsNodeHandle& myNh);

  /// @brief Get a connection handle previously added with addConnection
  ///
  /// Returns an empty shared pointer if the connection does not exist.
  ///
  /// \param name
  /// \return
  boost::shared_ptr<DsConnection> connection(const std::string& name);

  /// @brief Method to start all connections specified in the application
  ///
  /// @param[in] myNh A reference to the nodehandle object for accessing the parameter server
  /// @param[in] mapping An associative array that associates each connection name to a boost::function object that will
  /// be called with the data received when the connection identified by that name receives data
  ///
  /// @return An associative array that associates each connection name to a boost::shared_ptr object that is a handle
  /// for the created connection
  std::map<std::string, boost::shared_ptr<DsConnection> > startConnections(DsNodeHandle& myNh,
                                                                           std::map<std::string, ReadCallback> mapping);

  /// @brief Returns a pointer to this DsAsio instance
  ///
  /// @return A pointer to this DsAsio instance
  DsAsio* asio(void);

  /// @brief A custom signal handler that gracefully shuts down ROS before exiting the process
  void signalHandler(const boost::system::error_code& error, int signal_number);

  boost::asio::io_service io_service;

protected:
  std::map<std::string, ros::Subscriber> subs;
  std::map<std::string, ros::Timer> tmrs;
  std::map<std::string, ros::Publisher> pubs;

private:
  std::unordered_map<std::string, boost::shared_ptr<DsConnection> > connections;  //!< Map of active connections.
};
}

#endif
