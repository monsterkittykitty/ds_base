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
///   - `DsProcess::Impl::setupTimers`:       Add timers here.
///   - `DsProcess::Impl::setupServices`:     Add services here.
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
struct DsProcessPrivate
{
  DsProcessPrivate();
  virtual ~DsProcessPrivate() = default;

  /// @brief Handle restarting the status check timer.
  ///
  /// This is called from DsProcess::setStatusCheckPeriod and handles restarting
  /// the actual status check timer object if needed.
  ///
  /// \param base
  /// \param period
  void updateStatusCheckTimer(DsProcess* base, ros::Duration period);

  bool is_setup_;  //!< Has setup() been called?

  std::unique_ptr<ds_asio::DsAsio> asio_;               //!< DsAsio instance
  std::unique_ptr<ds_asio::DsNodeHandle> node_handle_;  //!< DsNodeHandle instance

  ros::Duration status_check_period_;  //!< The period for the status health timer (<0 disables)
  ros::Timer status_check_timer_;      //!< The status health timer itself.
  std::string descriptive_node_name_;  //!< A short, descriptive name given to the process.
  boost::uuids::uuid uuid_;            //!< UUID of node.

  ros::Publisher status_publisher_;  //!< The status channel publisher.

  std::unordered_map<std::string, ros::Time> last_published_timestamp_;  //!< Timestamp of last message sent by
                                                                         //!publisher
};
}
#endif  // DS_BASE_DS_PROCESS_PRIVATE_H
