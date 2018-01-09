#ifndef DS_PROCESS_H
#define DS_PROCESS_H

#include "ds_asio/ds_asio.h"
#include "ds_asio/ds_nodehandle.h"

#include <ros/ros.h>
#include <boost/asio.hpp>

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
  void addPublisher(const std::string& topic, uint32_t queue)
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
    publishers_[topic] = getNh()->advertise<T>(full_topic_name, queue);

    ROS_INFO_STREAM("New topic: " << full_topic_name);
  };

  /// @brief Get a publisher object created by addPublisher
  ///
  /// Note:  An invalid publisher is returned if no publisher exists for the provided topic.
  ///        This *shouldn't* crash things, ros::Publisher inclues it's own validity checks
  ///        before publishing, but they're all private methods and we're unable to access
  ///        them.
  ///
  /// \param topic
  /// \param valid  True if the returned ros::Publisher is valid.
  /// \return
  ros::Publisher publisher(const std::string& topic, bool *valid=nullptr)
  {
    auto it = publishers_.find(topic);
    if(it == publishers_.end()) {
      ROS_ERROR_STREAM("No publisher available for topic name: " << topic);
      if(valid != nullptr) {
        *valid = false;
      }
      return {};
    }

    if (valid != nullptr) {
      *valid = true;
    }

    return it->second;
  }

  /// @brief Add an asio-based (serial, udp, etc.) connection.
  ///
  /// \param name
  /// \param callback
  /// \return
  boost::shared_ptr<DsConnection> addConnection(const std::string& name, boost::function<void(ds_core_msgs::RawData)> callback);

  /// @brief Get a DsConnection object for a connection added previously by addConnection
  ///
  /// \param name
  /// \return
  boost::shared_ptr<DsConnection> connection(const std::string& name);

 protected:
  std::unique_ptr<DsAsio> myAsio;
  std::unique_ptr<ros::DsNodeHandle> nh;

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
  virtual void setup();

  /// @brief Get parameters from server
  ///
  /// The default implementation looks for the following PRIVATE parameters:
  ///  - health_check_period [double]
  ///  - descriptive_name [string]
  virtual void setupParameters();

  /// @brief Create asio connections
  virtual void setupConnections() {}

  /// @brief Create ros topic subscriptions.
  virtual void setupSubscriptions() {}

  /// @brief Create ros topic publishers.
  ///
  /// The default implementation creates the following publishers:
  ///  - status  [ds_core_msgs::Status]
  virtual void setupPublishers();
  std::unordered_map<std::string, ros::Publisher> publishers_;
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

  ros::Duration status_check_period_;   //!< The period for the status health timer (<0 disables)
  ros::Timer status_check_timer_;       //!< The status health timer itself.
  std::string descriptive_node_name_;   //!< A short, descriptive name given to the process.

};

#endif
