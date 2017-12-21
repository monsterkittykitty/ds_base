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

  /// @brief Default constructor
  DsAsio();

  /// @brief Constructor that calls ros::init(argc, argv, name)
  ///
  /// @param[in] argc Number of input command line arguments
  /// @param[in] argv Command line arguments
  /// @param[in] name Name of the ros process. may be remapped by ros::init if using roslaunch
  DsAsio(int argc, char** argv, const std::string &name);

  /// @brief Destructor
  ~DsAsio();

  /// @brief Run the owned boost::io_service event loop.
  ///
  /// This method blocks until terminated by signals.
  void run(void);

  /// @brief Method to add a single connection. Calls factory class to generate connections
  ///
  /// @param[in] name Name of the connection. Will be used to index the ros parameter server namespace
  /// @param[in] callback A boost::function object that will be called with the data received when the connection receives data
  /// @param[in] myNh A reference to the nodehandle object for accessing the parameter server
  ///
  /// @return A boost::shared_ptr object that is a handle for the created connection
  boost::shared_ptr<DsConnection> addConnection(std::string name, boost::function<void(ds_core_msgs::RawData)> callback, ros::DsNodeHandle& myNh);

  /// @brief Method to start all connections specified in the application
  ///
  /// @param[in] myNh A reference to the nodehandle object for accessing the parameter server
  /// @param[in] mapping An associative array that associates each connection name to a boost::function object that will be called with the data received when the connection identified by that name receives data
  ///
  /// @return An associative array that associates each connection name to a boost::shared_ptr object that is a handle for the created connection
  std::map<std::string, boost::shared_ptr<DsConnection> > startConnections(ros::DsNodeHandle& myNh, std::map<std::string, boost::function<void(ds_core_msgs::RawData data)> > mapping);

  /// @brief Returns a pointer to this DsAsio instance
  ///
  /// @return A pointer to this DsAsio instance
  DsAsio* asio(void);

  /// @brief Adds a ros::Subscriber object to the member subs associative array
  void addSub(std::string name, ros::Subscriber mySub);

  /// @brief Adds a ros::Timer object to the member tmrs associative array
  void addTmr(std::string name, ros::Timer myTmr);

  /// @brief Adds a ros::Publisher object to the member pubs associative array
  void addPub(std::string name, ros::Publisher myPub);

  /// @brief A custom signal handler that gracefully shuts down ROS before exiting the process
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
