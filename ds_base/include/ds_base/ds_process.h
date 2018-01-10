#ifndef DS_PROCESS_H
#define DS_PROCESS_H

#include "ds_asio/ds_asio.h"
#include "ds_asio/ds_nodehandle.h"

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/uuid/uuid.hpp>

namespace ds_base
{

/// @brief The base class for ROS nodes in Deep Submergence ROS
///
/// This class serves as the base class for all ROS nodes in the Deep Submergence
/// ROS ecosystem.
///
/// Reminder:  The `~` prefix before parameters and topics denotes "private" parameters
/// and topics.  That is, they live BELOW the fully resolved node name namespace.  So if
/// your node is named `node` in namespace `/some/namespace`, then the fully resolved node
/// namespace is `/some/namespace/node` and the parameter `~param` lives on the parameter
/// server at `/some/namespace/node/param`
///
/// # Parameters
///
/// `DsProcess` automatically checks the parameter server for the following:
///
///   - `~health_check_period`:  Period (in seconds) between checking internal status.
///   - `~descriptive_name`: A human-sensible name for the node.  Keep it short.
///   - `~uuid`: A unique, deterministic ID for the node.
///
/// # Topics
///
/// `DsProcess` automatically creates the following topics:
///
///   - `~status` (`ds_core_msgs::Status`):  Periodic status message.
///
/// # Extending
///
/// This class follows the `PIMPL` idiom, and all of the actual implementation details
/// are pushed into the protected `Impl` structure assocated with the class.  This
/// provides a stable ABI and a clean public interface for users.  These benifits come
/// at some extra cost in complexity.  A detailed guide to extending `DsProcess` can
/// be found in the `EXTENDING.md` document.
class DsProcess
{

 protected:
  struct Impl;

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
  /// @return A pointer to the protected DsNodeHandle instance. If the DsNodeHandle does not already exist,
  /// it in instantiated here
  ds_asio::DsNodeHandle* nodeHandle();

  /// @brief Run the owned asio io_service event loop.
  ///
  /// This method blocks until terminated by signals.
  void run();

  /// @brief Period betweeen process health checks.
  ///
  /// \return
  ros::Duration statusCheckPeriod() const noexcept;

  /// @brief Set the period for status health checks.
  ///
  /// A period < 0 disables health checks.
  void setStatusCheckPeriod(ros::Duration period) noexcept;

  /// @brief Give the node a descriptive name
  ///
  /// \param name
  void setDescriptiveName(const std::string& name) noexcept;

  /// @brief Get the node's descrpitive name;
  ///
  /// \return
  std::string descriptiveName() const noexcept;

  /// @brief Convenience function for adding a publisher
  ///
  /// Relative topic names (names not starting with '/') are appended to the node's name.  **HOWEVER!** The publisher
  /// object is STILL stored using the original passed topic.  Let's have a few examples:
  ///
  ///
  /// node name:  sensor
  /// topic name: data
  ///   --> messages will be published on 'sensor/data'
  ///   --> publisher can be retrieved by calling DsProcess::publisher('data')
  ///
  /// node name: sensor
  /// topic name: /absolute
  ///   --> messages will be published on '/absolute'
  ///   --> publisher can be retrieved by calling DsProcess::publisher('/absolute')
  ///
  /// \tparam T      ROS message type
  /// \param base    The owning SensorBase instance.
  /// \param topic   Name of the topic to publish on
  /// \param queue   Size of the publishing queue
  template <class T>
  void addPublisher(const std::string& topic, uint32_t queue)
  {
    // do nothing for empty strings...
    if (topic.empty())
    {
      return;
    }

    if (hasPublisher(topic))
    {
      ROS_ERROR_STREAM("Unable to add publisher named: " << topic << ". A publisher on that topic already exists.");
      return;
    }
    // construct our topic name and create the publisher.
    auto full_topic_name = topic.at(0) == '/' ? topic : ros::this_node::getName() + '/' + topic;
    auto pub = nodeHandle()->advertise<T>(full_topic_name, queue);

    // Store the publisher inside the impl.
    _addPublisher(topic, std::move(pub));

    ROS_INFO_STREAM("New topic: " << full_topic_name);
  }

  /// @brief Check if a publisher has been created with the provided name
  ///
  /// \param name
  /// \return
  bool hasPublisher(const std::string& name) const noexcept;

  /// @brief Get a publisher object created by addPublisher
  ///
  /// Note:  An invalid publisher is returned if no publisher exists for the provided topic.
  ///        This *shouldn't* crash things, ros::Publisher includes it's own validity checks
  ///        before publishing, but they're all private methods and we're unable to access
  ///        them.
  ///
  /// \param topic
  /// \param valid  True if the returned ros::Publisher is valid.
  /// \return
  ros::Publisher publisher(const std::string& topic, bool *valid=nullptr) const noexcept;

  /// @brief Add an asio-based (serial, udp, etc.) connection.
  ///
  /// \param name
  /// \param callback
  /// \return
  boost::shared_ptr<ds_asio::DsConnection> addConnection(const std::string& name, boost::function<void(ds_core_msgs::RawData)> callback);

  /// @brief Get a DsConnection object for a connection added previously by addConnection
  ///
  /// \param name
  /// \return
  boost::shared_ptr<ds_asio::DsConnection> connection(const std::string& name);

  /// @brief Get the UUID for the node.
  ///
  /// Generated UUID's are deterministic -- they should persist across runs
  /// with identical parameters.
  ///
  /// \return
  boost::uuids::uuid uuid() const noexcept;

 protected:

  /// @brief Construct a new DsProcess
  ///
  /// When using this constructor you must call ros::init elsewhere in your code
  DsProcess(std::unique_ptr<Impl> impl);

  /// @brief Construct a new DsProcess
  ///
  /// This constructor calls ros::init(argc, argv, name) for you
  ///
  /// @param[in] argc
  /// @param[in] argv
  /// @param[in] name The name of the process type
  DsProcess(std::unique_ptr<Impl> impl, int argc, char** argv, const std::string &name);

  /// @brief Access the underlying pimpl pointer.
  auto d_func() noexcept -> Impl*
  {
    return impl_.get();
  }

  /// @brief Access the underlying pimpl pointer.
  auto d_func() const noexcept -> Impl const*
  {
    return impl_.get();
  }

 private:
  // Called from DsProcess::addPublisher to store the created
  // ros::Publisher object inside the impl object.
  void _addPublisher(const std::string& name, ros::Publisher pub);

  std::shared_ptr<Impl> impl_;
};

}
#endif
