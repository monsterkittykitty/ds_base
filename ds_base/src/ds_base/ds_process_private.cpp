#include "ds_process_private.h"
#include "ds_base/ds_process.h"

#include "ds_core_msgs/Status.h"

#include <boost/uuid/nil_generator.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace ds_base
{
DsProcessPrivate::DsProcessPrivate()
  : asio_(std::unique_ptr<ds_asio::DsAsio>(new ds_asio::DsAsio))
  , uuid_(boost::uuids::nil_uuid())
  , is_setup_(false)
{
}

void DsProcessPrivate::updateStatusCheckTimer(DsProcess* base, ros::Duration period)
{
  if (period == status_check_period_)
  {
    return;
  }

  // Stop pending triggers.
  status_check_timer_.stop();

  // Negative durations disable the timer
  if (period < ros::Duration(0))
  {
    ROS_INFO_STREAM("Disabling status check timer");
    status_check_period_ = ros::Duration(-1);
    return;
  }

  status_check_period_ = period;
  status_check_timer_ = base->nodeHandle()->createTimer(status_check_period_, &DsProcess::checkProcessStatus, base);
  ROS_INFO_STREAM("Status check timer set to " << status_check_period_);
}
}