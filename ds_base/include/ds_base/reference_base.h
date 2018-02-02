#ifndef DS_BASE_REFERENCE_BASE_H
#define DS_BASE_REFERENCE_BASE_H

#include "ds_base/ds_process.h"
#include "ds_nav_msgs/AggregatedState.h"
#include "ds_param/ds_param_conn.h"

namespace ds_base
{

struct ReferenceBasePrivate;

/// @brief Base class for reference generators in DS_ROS
///
/// Closed loop control systems typically operate on the error
/// signal defined as the difference between the current state
/// of the system and the desired state.
///
/// Reference generators produce the "desired state" portion of
/// the equation
///
/// # Parameters
///
/// In addition to the parameters used in DsProcess, ReferenceBase looks
/// for the following:
///
/// - `~reference_output_topic`   Default: "reference_state".
///   Reference messages are published on this topic.
///
/// # Parameter Subscriptions
///
/// References all monitor the *namespace* integer parameter `active_reference`.
/// When this parameter changes the new value is checked against the value
/// returned by the `type()` method.  The controller is enabled if they match,
/// disabled if they don't.
///
/// # Published topics
///
/// - "reference_state" (ds_nav_msgs::AggregatedState)
///   Can be changed using the `~reference_output_topic` parameter
class ReferenceBase: public DsProcess
{
  DS_DECLARE_PRIVATE(ReferenceBase);
 public:

  /// @brief Construct a new DsProcess
  ///
  /// When using this constructor you must call ros::init elsewhere in your code
  ReferenceBase();

  /// @brief Construct a new DsReferenceBase
  ///
  /// This constructor calls ros::init(argc, argv, name) for you
  ///
  /// @param[in] argc
  /// @param[in] argv
  /// @param[in] name The name of the process type
  ReferenceBase(int argc, char** argv, const std::string& name);

  /// @brief Destroys a DsReferenceBase
  ~ReferenceBase() override;
  DS_DISABLE_COPY(ReferenceBase)

  /// @brief Unique type ID for this controller
  ///
  /// The type ID is used for two reasons:
  ///
  ///   - Eases downcasting a base class to the derived class
  ///   - Enables context switching using a ds_param subscription
  ///
  /// \return
  virtual uint64_t type() const noexcept = 0;

  /// @brief Set this reference active
  ///
  /// There should be only one reference active at a time.
  ///
  /// \param enabled
  virtual void setEnabled(bool enabled);

  /// @brief Get the active state of the reference.
  ///
  /// \return
  virtual bool enabled() const noexcept;

  /// @brief Publish a new reference state
  ///
  /// \param reference
  void publishReference(const ds_nav_msgs::AggregatedState& reference);
  void setup() override;
 protected:
  void setupPublishers() override;
  void setupSubscriptions() override;

  /// @brief Callback triggered by updates to subscribed parameters.
  ///
  /// The default behavior enables/disables the reference based on the value
  /// of the `active_reference` parameter
  ///
  /// \param params
  virtual void parameterSubscriptionCallback(const ds_param::ParamConnection::ParamCollection& params);

 private:
  std::unique_ptr<ReferenceBasePrivate> d_ptr_;

};

}
#endif //DS_BASE_REFERENCE_BASE_H
