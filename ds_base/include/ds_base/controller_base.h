#ifndef DS_BASE_CONTROLLERBASE_H
#define DS_BASE_CONTROLLERBASE_H

#include "ds_base/ds_process.h"
#include "ds_nav_msgs/AggregatedState.h"

namespace ds_base
{

struct ControllerBasePrivate;

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
  void setEnabled(bool enabled);

  /// @brief Is controller output enabled?
  ///
  /// \return
  bool enabled() const noexcept;

 protected:

  /// @brief
  virtual void stateUpdateCallback(const ds_nav_msgs::AggregatedState& msg)
  {
  }

 private:
  std::unique_ptr<ControllerBasePrivate> d_ptr_;
};

}
#endif //DS_BASE_CONTROLLERBASE_H
