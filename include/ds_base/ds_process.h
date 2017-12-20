#ifndef DS_PROCESS_H
#define DS_PROCESS_H

#include "ds_base/ds_asio.h"
#include <ros/ros.h>
#include <boost/asio.hpp>
#include "ds_base/ds_nodehandle.h"

class DsProcess
{

public:
  /// @brief Construct a new DsProcess
  ///
  /// When using this constructor you must call ros::init elsewhere in your code
  DsProcess();

  /// @biref Construct a new DsProcess
  ///
  /// This constructor calls ros::init(argc, argv, name) for you
  ///
  /// \param argc
  /// \param argv
  /// \param name
  DsProcess(int argc, char** argv, const std::string &name);

  virtual ~DsProcess();

  /// @brief Access the owned DsNodeHandle
  ros::DsNodeHandle* getNh();

  /// @brief Run the owned asio io_service event loop.
  ///
  /// This method blocks until terminated by signals.
  void run();

protected:
  std::unique_ptr<ros::DsNodeHandle> nh;
  std::unique_ptr<DsAsio> myAsio;

};

#endif
