#ifndef DS_BASE_REFERENCE_BASE_H
#define DS_BASE_REFERENCE_BASE_H

#include "ds_base/ds_process.h"
#include "ds_nav_msgs/AggregatedState.h"

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


  /// @brief Set this reference active
  ///
  /// There should be only one reference active at a time.
  ///
  /// \param enabled
  virtual void setEnabled(bool enabled);

  /// @brief Get the active state of the reference.
  ///
  /// \return
  bool enabled() const noexcept;

  /// @brief Publish a new reference state
  ///
  /// \param reference
  void publishReference(const ds_nav_msgs::AggregatedState& reference);

 protected:
  void setupPublishers() override;

 private:
  std::unique_ptr<ReferenceBasePrivate> d_ptr_;

};

}
#endif //DS_BASE_REFERENCE_BASE_H
