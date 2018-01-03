#include "ds_base/ds_process.h"


DsProcess::DsProcess()
  : myAsio(std::unique_ptr<DsAsio>(new DsAsio()))
  , status_check_period_(ros::Duration(-1))
{
}

DsProcess::DsProcess(int argc, char** argv, const std::string &name)
  : myAsio(std::unique_ptr<DsAsio>(new DsAsio(argc, argv, name)))
  , nh(std::unique_ptr<ros::DsNodeHandle>(new ros::DsNodeHandle(&myAsio->io_service)))
  , status_check_period_(ros::Duration(-1))
{
}

DsProcess::~DsProcess() = default;


ros::DsNodeHandle* DsProcess::getNh()
{
  if(!nh)
    {
      nh.reset(new ros::DsNodeHandle(&(myAsio->io_service)));
    }

  return nh.get();
}

void DsProcess::run()
{
  if(status_check_timer_.isValid()) {
    status_check_timer_.start();
  }
  myAsio->run();
  status_check_timer_.stop();
}

ros::Duration DsProcess::statusCheckPeriod() const noexcept
{
  return status_check_period_;
}

void DsProcess::setStatusCheckPeriod(ros::Duration period) noexcept
{
  if(period == status_check_period_) {
    return;
  }

  // Stop pending triggers.
  status_check_timer_.stop();

  // Negative durations disable the timer
  if(period < ros::Duration(0)) {
    ROS_INFO_STREAM("Disabling status check timer");
    status_check_period_ = ros::Duration(-1);
    return;
  }

  status_check_period_ = period;
  // Create a new periodic, auto-starting timer using the nodehandle with our asio callback queue
  status_check_timer_ = getNh()->createTimer(status_check_period_, &DsProcess::checkProcessStatus, this);
  ROS_INFO_STREAM("Status check timer set to " << status_check_period_);
}
