//
// Created by ivaughn on 1/15/18.
//

#ifndef PROJECT_DS_BUS_DEVICE_PRIVATE_H
#define PROJECT_DS_BUS_DEVICE_PRIVATE_H

#include "ds_base/ds_bus_device.h"
#include "ds_base/ds_process_private.h"
#include "ds_process.h"
#include <ds_base/util.h>

#include <ds_core_msgs/Status.h>
#include <ds_core_msgs/RawData.h>
#include <ds_core_msgs/IoSMcommand.h>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/nil_generator.hpp>

namespace ds_base
{
struct DsBusDevice::Impl : public ds_base::DsProcess::Impl
{
  Impl();

  virtual ~Impl() = default;

  // disable copy operations
  Impl(const Impl&) = delete;
  void operator=(const Impl&) = delete;

  /// @brief Convenience method to send an I/O state machine command and get the reply
  ds_core_msgs::IoSMcommand::Response sendIosmCommand(const ds_core_msgs::IoSMcommand::Request& cmd);

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

    last_message_[name] = msg;
    last_message_timestamp_[name] = msg.header.stamp;
  }

  boost::uuids::uuid uuid_;

  std::unordered_map<std::string, ros::Publisher> publishers_;         //!< Collection of data message publishers
  std::unordered_map<std::string, boost::any> last_message_;           //!< Last message published on given topic
  std::unordered_map<std::string, ros::Time> last_message_timestamp_;  //!< Last message timestamp.

  ros::Duration message_timeout_;  //!< Time between valid messages to consider bad.

  /// \brief The name of the node that's managing this bus
  std::string bus_node_name_;

  /// \brief The topic that spits all bus traffic
  ros::Subscriber bus_;

  /// \brief The service that controls the I/O state machine on that bus
  ros::ServiceClient iosm_cmd_;
};
}  // namespace ds_base

#endif  // PROJECT_DS_BUS_DEVICE_PRIVATE_H
