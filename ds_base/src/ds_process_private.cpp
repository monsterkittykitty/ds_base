#include "ds_base/ds_process_private.h"
#include "ds_base/ds_process.h"

#include "ds_core_msgs/Status.h"

namespace ds_base
{

DsProcess::Impl::Impl()
  : asio_(std::unique_ptr<ds_asio::DsAsio>(new ds_asio::DsAsio))
{
}

void DsProcess::Impl::setup(DsProcess* base)
{
  setupParameters(base);
  setupConnections(base);
  setupSubscriptions(base);
  setupPublishers(base);
}


void DsProcess::Impl::setupParameters(DsProcess* base)
{
  const auto health_check_period = ros::param::param<double>("~health_check_period", -1.0);
  if (health_check_period > 0) {
    ROS_INFO_STREAM("Setting status updated period to " << health_check_period << " seconds.");
  }
  else {
    ROS_INFO_STREAM("Disabling periodic status checks.");
  }
  base->setStatusCheckPeriod(ros::Duration(health_check_period));
  const auto name_ = ros::param::param<std::string>("~descriptive_name", "NO_NAME_PROVIDED");
  base->setDescriptiveName(name_);
}

void DsProcess::Impl::setupPublishers(DsProcess* base)
{
  base->addPublisher<ds_core_msgs::Status>("status", 10);
}

void DsProcess::Impl::updateStatusCheckTimer(DsProcess* base, ros::Duration period)
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
  status_check_timer_ = base->nodeHandle()->createTimer(status_check_period_, &DsProcess::Impl::checkProcessStatus, this);
  ROS_INFO_STREAM("Status check timer set to " << status_check_period_);
}

}