#include "ds_base/ds_process_private.h"
#include "ds_base/ds_process.h"

#include "ds_core_msgs/Status.h"

namespace ds_base
{

DsProcess::Impl::Impl()
  : myAsio(std::unique_ptr<ds_asio::DsAsio>(new ds_asio::DsAsio))
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

}