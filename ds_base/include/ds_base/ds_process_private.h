//
// Created by zac on 1/9/18.
//

#ifndef DS_BASE_DS_PROCESS_PRIVATE_H
#define DS_BASE_DS_PROCESS_PRIVATE_H

#include "ds_asio/ds_asio.h"
#include "ds_asio/ds_nodehandle.h"
#include "ds_base/ds_process.h"

namespace ds_base
{

struct DsProcess::Impl
{

  Impl();
  virtual ~Impl() = default;

  /// @brief Setup node after ros has been initialized.
  ///
  /// This method is called in DsProcess' constructors, after the object has
  /// been instantiated.  It is the main entry point to add run-time configuration
  /// that requires a rosmaster to be running.
  ///
  /// Default implmementation calls, in order:
  ///
  /// - setupParameters()
  /// - setupConnections()
  /// - setupSubscriptions()
  /// - setupPublishers()
  ///
  virtual void setup(DsProcess* base);

  /// @brief Get parameters from server
  ///
  /// The default implementation looks for the following PRIVATE parameters:
  ///  - health_check_period [double]
  ///  - descriptive_name [string]
  virtual void setupParameters(DsProcess* base);

  /// @brief Create asio connections
  virtual void setupConnections(DsProcess* base) {}

  /// @brief Create ros topic subscriptions.
  virtual void setupSubscriptions(DsProcess* base) {}

  /// @brief Create ros topic publishers.
  ///
  /// The default implementation creates the following publishers:
  ///  - status  [ds_core_msgs::Status]
  virtual void setupPublishers(DsProcess* base);

  /// @brief Check the process status.
  ///
  /// This method is triggered by the status check timer.  The default
  /// implementation does nothing.
  ///
  /// This is where you can add hooks to check process-specific details
  /// and emit a ds_core_msgs::Status message.
  ///
  /// \param event
  virtual void checkProcessStatus(const ros::TimerEvent &event) {};


  void updateStatusCheckTimer(DsProcess* base, ros::Duration period);

  std::unique_ptr<ds_asio::DsAsio> myAsio;
  std::unique_ptr<ds_asio::DsNodeHandle> nh;

  std::unordered_map<std::string, ros::Publisher> publishers_;

  ros::Duration status_check_period_;   //!< The period for the status health timer (<0 disables)
  ros::Timer status_check_timer_;       //!< The status health timer itself.
  std::string descriptive_node_name_;   //!< A short, descriptive name given to the process.

};

}
#endif //DS_BASE_DS_PROCESS_PRIVATE_H
