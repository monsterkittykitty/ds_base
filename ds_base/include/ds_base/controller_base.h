#ifndef DS_BASE_CONTROLLERBASE_H
#define DS_BASE_CONTROLLERBASE_H

#include "ds_base/ds_process.h"
#include "ds_nav_msgs/AggregatedState.h"

namespace ds_base
{

struct ControllerBasePrivate;

/// @brief Base class for controllers.
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
  /// \param msg
  virtual void setReference(const ds_nav_msgs::AggregatedState &msg);
  const ds_nav_msgs::AggregatedState& reference() const noexcept;

  /// @brief  Callback fired upon receipt of new state messages
  ///
  /// \param msg
  virtual void setState(const ds_nav_msgs::AggregatedState &msg);
  const ds_nav_msgs::AggregatedState& state() const noexcept;


 protected:
  void setupSubscriptions() override;

 private:
  std::unique_ptr<ControllerBasePrivate> d_ptr_;
};

}
#endif //DS_BASE_CONTROLLERBASE_H
