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
struct DsProcess::Impl
{
  Impl();
  virtual ~Impl() = default;

  /// @brief Handle restarting the status check timer.
  ///
  /// This is called from DsProcess::setStatusCheckPeriod and handles restarting
  /// the actual status check timer object if needed.
  ///
  /// \param base
  /// \param period
  void updateStatusCheckTimer(DsProcess* base, ros::Duration period);

  /// @brief Create a data publisher
  ///
  /// Provided topic names will have the node name prepended to it, creating a "private"
  /// scoped topic name.
  ///
  /// \tparam T     Type of message to publish
  /// \param name   name of topic.  Will be "private" scope (e.g. beneath the node name)
  /// \param queue  Size of topic queue
  /// \param latch  Data topic is latching or not.
  template <typename T>
  void addMessagePublisher(const std::string& name, uint32_t queue, bool latch = false)
  {
    publishers_[name] = node_handle_->advertise<T>(ros::this_node::getName() + "/" + name, queue, latch);
  }

  /// @brief Create a topic subscriber
  ///
  /// \tparam T     Type of message to publish
  /// \param name   name of topic.  Will be "private" scope (e.g. beneath the node name)
  /// \param queue  Size of topic queue
  /// \param latch  Data topic is latching or not.
  template <typename T>
  void addMessageSubscriber(const std::string& name, uint32_t queue,
                            const boost::function<void(const boost::shared_ptr<T const>&)>& callback,
                            const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
                            const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    // Using the DsProcess public interface
    subscribers_[name] = node_handle_->subscribe(name, queue, callback, tracked_object, transport_hints);
  }

  void addTimer(std::string name, ros::Duration interval, boost::function<void(const ros::TimerEvent&)> callback)
  {
    timers_[name] = node_handle_->createTimer(interval, callback);
  }

  template <typename T>
  void publishMessage(const std::string& name, T msg)
  {
    auto ok = false;
    try
    {
      auto& pub = publishers_.at(name);
      if (!pub)
      {
        ROS_WARN_STREAM("Unable to publish on topic: " << name << ".  ros::Publisher is invalid");
        return;
      }
      pub.publish(msg);
    }
    catch (std::out_of_range& e)
    {
      ROS_WARN_STREAM("Unable to publish on topic: " << name << ".  No publisher found for topic");
      return;
    }

    last_published_timestamp_[name] = msg.header.stamp;
  }

  /// @brief Copy timestamps and uuid into a new message
  ///
  /// Convenience method for copying timestamps from a raw data message and injecting
  /// the sensor uuid
  ///
  /// \tparam T
  /// \param output
  /// \param bytes
  template <typename T>
  void copyMessageHeader(T& output, const ds_core_msgs::RawData& bytes)
  {
    output.ds_header.io_time = bytes.ds_header.io_time;
    std::copy(std::begin(output.ds_header.source_uuid), std::end(output.ds_header.source_uuid), std::begin(uuid_.data));
    output.header.stamp = output.ds_header.io_time;
  }

  bool is_setup_;                                                 //!< Has setup() been called?
  std::unordered_map<std::string, ros::Publisher> publishers_;    //!< Collection of data message publishers
  std::unordered_map<std::string, ros::Subscriber> subscribers_;  //!< Collection of data message publishers
  std::unordered_map<std::string, ros::Timer> timers_;            //!< Collection of timers

  std::unordered_map<std::string, ros::Time> last_published_timestamp_;  //!< Timestamp of last message sent by
                                                                         //!publisher

  std::unique_ptr<ds_asio::DsAsio> asio_;               //!< DsAsio instance
  std::unique_ptr<ds_asio::DsNodeHandle> node_handle_;  //!< DsNodeHandle instance

  ros::Duration status_check_period_;  //!< The period for the status health timer (<0 disables)
  ros::Timer status_check_timer_;      //!< The status health timer itself.
  std::string descriptive_node_name_;  //!< A short, descriptive name given to the process.
  boost::uuids::uuid uuid_;            //!< UUID of node.

  ros::Duration message_timeout_;  //!< Time between valid messages to consider bad.

  ros::Publisher status_publisher_;  //!< The status channel publisher.
};
}
#endif  // DS_BASE_DS_PROCESS_PRIVATE_H
