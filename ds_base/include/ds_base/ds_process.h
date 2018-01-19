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
  /// This is just one of many of the overloads for adding a publisher to a ros::NodeHandle object.
  /// Consult the ros::NodeHandle documentation for a complete list.  The ROS documenation for this
  /// specific overload is provided below verbatim:
  ///
  /// This call connects to the master to publicize that the node will be
  /// publishing messages on the given topic.  This method returns a Publisher that allows you to
  /// publish a message on this topic.
  ///
  /// This version of advertise is a templated convenience function, and can be used like so
  ///
  ///   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
  ///
  /// \param topic Topic to advertise on
  ///
  /// \param queue_size Maximum number of outgoing messages to be
  /// queued for delivery to subscribers
  ///
  /// \param latch (optional) If true, the last message published on
  /// this topic will be saved and sent to new subscribers when they
  /// connect
  ///
  /// \return On success, a Publisher that, when it goes out of scope,
  /// will automatically release a reference on this advertisement.  On
  /// failure, an empty Publisher.
  ///
  /// \throws InvalidNameException If the topic name begins with a
  /// tilde, or is an otherwise invalid graph resource name, or is an
  /// otherwise invalid graph resource name
  ///
  template <class T>
  ros::Publisher advertise(const std::string& topic, uint32_t queue, bool latch=false)
  {
    return nodeHandle()->advertise<T>(topic, queue, latch);
  }

  /// @brief Convenience function for adding a subscriber
  ///
  /// This is just one of many of the overloads for adding a subscriber to a ros::NodeHandle object.
  /// Consult the ros::NodeHandle documentation for a complete list.  The ROS documenation for this
  /// specific overload is provided below verbatim:
  ///
  /// This method connects to the master to register interest in a given
  /// topic.  The node will automatically be connected with publishers on
  /// this topic.  On each message receipt, callback is invoked and passed a shared pointer
  /// to the message received.  This message should \b not be changed in place, as it
  /// is shared with any other subscriptions to this topic.
  ///
  /// This version of subscribe allows anything bindable to a boost::function object
  ///
  /// \param T [template] M here is the message type
  /// \param topic Topic to subscribe to
  /// \param queue_size Number of incoming messages to queue up for
  /// processing (messages in excess of this queue capacity will be
  /// discarded).
  /// \param callback Callback to call when a message has arrived
  /// \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
  /// and if the reference count goes to 0 the subscriber callbacks will not get called.
  /// Note that setting this will cause a new reference to be added to the object before the
  /// callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
  /// thread) that the callback is invoked from.
  /// \param transport_hints a TransportHints structure which defines various transport-related options
  /// \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
  /// On failure, an empty Subscriber which can be checked with:
  /// \verbatim
  ///      void callback(const std_msgs::Empty::ConstPtr& message){...}
  ///      ros::NodeHandle nodeHandle;
  ///      ros::Subscriber sub = nodeHandle.subscribe("my_topic", 1, callback);
  ///      if (sub)  // Enter if subscriber is valid
  ///      {
  ///      ...
  ///      }
  /// \endverbatim
  /// \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
  /// \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
  template <class T>
  ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                     const boost::function< void(const boost::shared_ptr< T const > &)> & callback,
                     const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
                     const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    return nodeHandle()->subscribe<T>(topic, queue_size, callback, tracked_object, transport_hints);
  }

  template<class M, class T>
  ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                            void(T::*fp)(M), T *obj,
                            const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    return nodeHandle()->subscribe<M,T>(topic, queue_size, fp, obj, transport_hints);
  };

  /// @brief Conveinence method for connecting to a ROS service as a client
  ///
  /// As with the other wrappers, we only wrap a single overload.  You can DEFINITELY use others
  /// by grabbing the node handle directly.
  ///
  /// Note that this call will return even for services that don't exist yet.  You are
  /// STRONGLY encouraged to check out ros::ServiceClient::waitForExistence
  ///
  /// \tparam Service The srv type for this message.  Basically just the name of the .srv file.
  /// \param service_name The name of the service to connect to
  /// \param persistent Make this a persistent connection.  Generally discouraged.
  /// \param header_values Key/value pairs to send with the connection handshake.
  /// \return A service client object for this service.  Note that this service may not exist yet!
  template <class Service>
  ros::ServiceClient serviceClient(const std::string& service_name, bool persistent=false, const ros::M_string& header_values=ros::M_string()) {
    return nodeHandle()->serviceClient<Service>(service_name, persistent, header_values);
  }

  /// @brief Convenience function for adding a service
  ///
  /// This is just one of many of the overloads for adding a service to a ros::NodeHandle object.
  /// Consult the ros::NodeHandle documentation for a complete list.  The ROS documenation for this
  /// specific overload is provided below verbatim:
  ///
  /// This call connects to the master to publicize that the node will be
  /// offering an RPC service with the given name.
  ///
  /// This version of advertiseService allows non-class functions, as well as functor objects and boost::bind (along with anything
  /// else boost::function supports).
  ///
  /// \param service Service name to advertise on
  /// \param callback Callback to call when the service is called
  /// \param tracked_object A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
  /// and if the reference count goes to 0 the subscriber callbacks will not get called.
  /// Note that setting this will cause a new reference to be added to the object before the
  /// callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
  /// thread) that the callback is invoked from.
  /// \return On success, a ServiceServer that, when all copies of it go out of scope, will unadvertise this service.
  /// On failure, an empty ServiceServer which can be checked with:
  /// \verbatim
  /// bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
  /// {
  ///   return true;
  /// }
  /// ros::NodeHandle nodeHandle;
  /// Foo foo_object;
  /// ros::ServiceServer service = nodeHandle.advertiseService("my_service", callback);
  /// if (service)  // Enter if advertised service is valid
  /// {
  /// ...
  /// }
  /// \endverbatim
  /// \throws InvalidNameException If the service name begins with a tilde, or is an otherwise invalid graph resource name
  ///
  template <class MReq, class MRes>
  ros::ServiceServer advertiseService(const std::string &  	service,
		const boost::function< bool(MReq &, MRes &)> &  	callback,
		const ros::VoidConstPtr &  	tracked_object = ros::VoidConstPtr())
  {
    return nodeHandle()->advertiseService<MReq, MRes>(service, callback, tracked_object);
  }
  /// @brief Convenience method of creating a timer
  ///
  /// This is just one of many of the overloads for adding a timer to a ros::NodeHandle object.
  /// Consult the ros::NodeHandle documentation for a complete list.  The ROS documenation for this
  /// specific overload is provided below verbatim:
  ///
  /// \brief Create a timer which will call a callback at the specified rate.  This variant takes
  /// anything that can be bound to a Boost.Function, including a bare function
  ///
  /// When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
  /// be stopped, and the callback will no longer be called.
  ///
  /// \param period The period at which to call the callback
  /// \param callback The function to call
  /// \param oneshot If true, this timer will only fire once
  /// \param autostart If true (default), return timer that is already started
  ///
  ros:: Timer createTimer(ros::Duration period, const ros::TimerCallback& callback, bool oneshot = false, bool autostart = true)
  {
    return nodeHandle()->createTimer(period, callback, oneshot, autostart);
  }

  /// @brief Add an asio-based (serial, udp, etc.) connection.
  ///
  /// \param name
  /// \param callback
  /// \return
  boost::shared_ptr<ds_asio::DsConnection> addConnection(const std::string& name, ds_asio::DsAsio::ReadCallback callback);

    /// @brief Add an asio-based I/O state machine and its associated connection (serial, UDP, etc)
    ///
    /// \param iosm_name The name of the I/O state machine (for parameter stuff)
    /// \param conn_name The name of the connection (also for parameter stuff)
    /// \param callback The callback fired when the state machine has data to send
    /// \return A shared_ptr to the I/O state machine object
    boost::shared_ptr<ds_asio::IoSM> addIoSM(const std::string& iosm_name, const std::string& conn_name, ds_asio::DsAsio::ReadCallback callback);

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
  /// - setupTimers()
  /// - setupServices()
  ///
  virtual void setup();

  /// @brief Get parameters from server
  ///
  /// The default implementation looks for the following PRIVATE parameters:
  ///  - health_check_period [double]
  ///  - descriptive_name [string]
  ///  - uuid [string]
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

  /// @brief Create ros services
  ///
  virtual void setupServices() {}

  /// @brief Create ros timers on startup.
  ///
  /// The default implementation does nothing.
  ///
  /// \param base

  virtual void setupTimers() {}
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
 private:

  std::shared_ptr<Impl> impl_;
};

}
#endif
