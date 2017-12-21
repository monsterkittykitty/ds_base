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

  /// @brief Construct a new DsProcess
  ///
  /// This constructor calls ros::init(argc, argv, name) for you
  ///
  /// @param[in] argc
  /// @param[in] argv
  /// @param[in] name The name of the process type
  DsProcess(int argc, char** argv, const std::string &name);

  /// @brief Destroys a DsProcess
  virtual ~DsProcess();

  /// @brief Access the owned DsNodeHandle
  ///
  /// @return A pointer to the protected DsNodeHandle instance. If the DsNodeHandle does not already exist, it in instantiated here
  ros::DsNodeHandle* getNh();

  /// @brief Run the owned asio io_service event loop.
  ///
  /// This method blocks until terminated by signals.
  void run();

protected:
  std::unique_ptr<DsAsio> myAsio;
  std::unique_ptr<ros::DsNodeHandle> nh;

};

#endif
