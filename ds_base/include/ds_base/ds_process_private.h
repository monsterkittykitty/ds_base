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

/// @brief The base implementation class for ROS nodes in Deep Submergence ROS.
///
/// This class provides the core implementation details for all `DsProcess-based`
/// nodes.  At the core of this are a number of virtual methods that allow users
/// to add custom steps where needed in their node's setup:
///
///   - `DsProcess::Impl::setupParameters`:   Parameter-server based setup directives.
///   - `DsProcess::Impl::setupConnections`:  Add `DsAsio` connections here
///   - `DsProcess::Impl::setupPublishers`:   Add topic publishers here.
///   - `DsProcess::Impl::setupSubscribers`:  Add subscribers here.
///   - `DsProcess::Impl::setup`:             Wraps all of the above
///
/// This allows developers to selectively add extra configuration directives without
/// re-implementing by hand all of the parent class' stuff by hand.
///
/// **NOTE:** You **don't** have to call setup() in your own constructors.  That's the whole
/// point of this.  The base class will do it for you.
///
/// # What Goes In Here?
///
/// This class (and those derived from it) should contain as much of the non-public
/// implementation details for their associated public class.   Examples of things
/// you'll want to put here:
///
///   - Helper methods not intended to be called by users
///   - Member variables not intended to be accessed by users
///   - ... really anything you'd mark `protected:` or `private:`
///
/// There are times when you can't follow this strictly, for example if you want to
/// use some templated functions.  But try to keep as much of the details "hidden"
/// from the user.  Besides presenting a cleaner object for use, it helps keep
/// ABI promises.
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
  ///  - uuid [string]
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


  /// @brief Handle restarting the status check timer.
  ///
  /// This is called from DsProcess::setStatusCheckPeriod and handles restarting
  /// the actual status check timer object if needed.
  ///
  /// \param base
  /// \param period
  void updateStatusCheckTimer(DsProcess* base, ros::Duration period);

  std::unique_ptr<ds_asio::DsAsio> asio_;  //!< DsAsio instance
  std::unique_ptr<ds_asio::DsNodeHandle> node_handle_; //!< DsNodeHandle instance

  std::unordered_map<std::string, ros::Publisher> publishers_; //!< Topic/Publisher pairings

  ros::Duration status_check_period_;   //!< The period for the status health timer (<0 disables)
  ros::Timer status_check_timer_;       //!< The status health timer itself.
  std::string descriptive_node_name_;   //!< A short, descriptive name given to the process.
  boost::uuids::uuid uuid_;             //!< UUID of node.

};

}
#endif //DS_BASE_DS_PROCESS_PRIVATE_H
