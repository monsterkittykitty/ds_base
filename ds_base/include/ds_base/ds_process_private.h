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

  Impl() = default;
  virtual ~Impl() = default;

  /// @brief Convenience function for adding a publisher
  ///
  /// Relative topic names (names not starting with '/') are appended to the node's name.  **HOWEVER!** The publisher
  /// object is STILL stored using the original passed topic.  Let's have a few examples:
  ///
  ///
  /// node name:  sensor
  /// topic name: data
  ///   --> messages will be published on 'sensor/data'
  ///   --> publisher can be retrieved by getting publisher_.at('data')
  ///
  /// node name: sensor
  /// topic name: /absolute
  ///   --> messages will be published on '/absolute'
  ///   --> publisher can be retrieved by getting publisher_at('/absolute')
  ///
  /// \tparam T      ROS message type
  /// \param base    The owning SensorBase instance.
  /// \param topic   Name of the topic to publish on
  /// \param queue   Size of the publishing queue
  template <class T>
  void addPublisher(DsProcess* base, const std::string& topic, uint32_t queue)
  {
    // do nothing for empty strings...
    if (topic.empty())
    {
      return;
    }

    if (publishers_.find(topic) != publishers_.end()) {
      ROS_ERROR_STREAM("Unable to add publisher named: " << topic << ". A publisher on that topic already exists.");
      return;
    }

    // construct our topic name and create the publisher.
    auto full_topic_name = topic.at(0) == '/' ? topic : ros::this_node::getName() + '/' + topic;
    publishers_[topic] = base->getNh()->advertise<T>(full_topic_name, queue);

    ROS_INFO_STREAM("New topic: " << full_topic_name);
  };

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

  std::unique_ptr<ds_asio::DsAsio> myAsio;
  std::unique_ptr<ds_asio::DsNodeHandle> nh;

  std::unordered_map<std::string, ros::Publisher> publishers_;

  ros::Duration status_check_period_;   //!< The period for the status health timer (<0 disables)
  ros::Timer status_check_timer_;       //!< The status health timer itself.
  std::string descriptive_node_name_;   //!< A short, descriptive name given to the process.

};

}
#endif //DS_BASE_DS_PROCESS_PRIVATE_H
