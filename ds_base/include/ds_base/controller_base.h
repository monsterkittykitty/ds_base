#ifndef DS_BASE_CONTROLLERBASE_H
#define DS_BASE_CONTROLLERBASE_H

#include "ds_base/ds_process.h"
#include "ds_nav_msgs/AggregatedState.h"
#include "ds_param/ds_param_conn.h"

namespace ds_base
{

struct ControllerBasePrivate;

/// @brief Base class for controllers.
///
/// # Parameters
///
/// `ControllerBase` looks for the following parameters:
///
///   `~state_input_topic`  defaults to `state_input`
///   `~reference_input_topic` defaults to `reference_input`
///
/// # Parameter Subscriptions
///
/// References all monitor the *namespace* integer parameter `active_reference`.
/// When this parameter changes the new value is checked against the value
/// returned by the `type()` method.  The controller is enabled if they match,
/// disabled if they don't.
///
/// # Topics
///
/// `ControllerBase` automatically subscribes to the following topics:
///
///   - `state_input`     (`ds_nav_msgs::AggregatedState`):  State update messages
///   - `reference_input` (`ds_nav_msgs::AggregatedState`):  Reference update messages
class ControllerBase : public ds_base::DsProcess
{
  DS_DECLARE_PRIVATE(ControllerBase)

 public:

  ///@brief ControllerBase constructor
  ///
  /// You must call `ros::init` separately when using this method.
  explicit ControllerBase();

  /// @brief ControllerBase constructor
  ///
  /// Calls ros::init(argc, argv, name) for you.
  ///
  /// \param argc
  /// \param argv
  /// \param name
  ControllerBase(int argc, char* argv[], const std::string& name);
  ~ControllerBase() override;

  DS_DISABLE_COPY(ControllerBase);

  /// @brief Unique type ID for this controller
  ///
  /// The type ID is used for two reasons:
  ///
  ///   - Eases downcasting a base class to the derived class
  ///   - Enables context switching using a ds_param subscription
  ///
  /// \return
  virtual uint64_t type() const noexcept = 0;

  /// @brief Enable controller output
  ///
  /// \param enabled
  virtual void setEnabled(bool enabled);

  /// @brief Is controller output enabled?
  ///
  /// \return
  virtual bool enabled() const noexcept;

  /// @brief Callback fired upon receipt of new reference messages
  ///
  /// Default implementation stores the reference message, and can be
  /// retrieved using the `reference()` method
  ///
  /// \param msg
  virtual void setReference(const ds_nav_msgs::AggregatedState &msg);

  /// @breif Retrieve the last recorded reference message.
  ///
  /// Returns the last reference message stored using `setReference()`
  ///
  /// \return
  const ds_nav_msgs::AggregatedState& reference() const noexcept;

  /// @brief  Callback fired upon receipt of new state messages
  ///
  /// Default implementation stores the state message, and can be
  /// retrieved using the `state()` method
  ///
  /// \param msg
  virtual void setState(const ds_nav_msgs::AggregatedState &msg);

  /// @breif Retrieve the last recorded state message.
  ///
  /// Returns the last state message stored using `setState()`
  ///
  /// \return
  const ds_nav_msgs::AggregatedState& state() const noexcept;

  void setup() override;
 protected:
  void setupSubscriptions() override;

  /// @brief Callback triggered by updates to subscribed parameters.
  ///
  /// The default behavior enables/disables the reference based on the value
  /// of the `active_reference` parameter
  ///
  /// \param params
  virtual void parameterSubscriptionCallback(const ds_param::ParamConnection::ParamCollection& params);

 private:
  std::unique_ptr<ControllerBasePrivate> d_ptr_;
};

}
#endif //DS_BASE_CONTROLLERBASE_H
