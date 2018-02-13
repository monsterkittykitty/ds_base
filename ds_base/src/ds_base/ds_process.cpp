#include "ds_base/ds_process.h"
#include "ds_process_private.h"
#include "ds_core_msgs/Status.h"

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace ds_base
{
DsProcess::DsProcess() : d_ptr_(std::unique_ptr<DsProcessPrivate>(new DsProcessPrivate))
{
}

DsProcess::DsProcess(int argc, char** argv, const std::string& name)
  : d_ptr_(std::unique_ptr<DsProcessPrivate>(new DsProcessPrivate))
{
  ros::init(argc, argv, name);
}

DsProcess::~DsProcess() = default;

ros::NodeHandle DsProcess::nodeHandle(const std::string& ns)
{
  DS_D(DsProcess);
  auto nh = ros::NodeHandle(ns);
  nh.setCallbackQueue(d->asio_->callbackQueue());

  return nh;
}

void DsProcess::run()
{
  DS_D(DsProcess);
  if (!d->is_setup_)
  {
    setup();
  }

  if (d->status_check_timer_.isValid())
  {
    d->status_check_timer_.start();
  }
  d->asio_->run();
  d->status_check_timer_.stop();
}

void DsProcess::setDescriptiveName(const std::string& name) noexcept
{
  ROS_INFO_STREAM("Setting descriptive name to: " << name);
  DS_D(DsProcess);
  d->descriptive_node_name_ = name;
}

std::string DsProcess::descriptiveName() const noexcept
{
  const DS_D(DsProcess);
  return d->descriptive_node_name_;
}

ros::Duration DsProcess::statusCheckPeriod() const noexcept
{
  const DS_D(DsProcess);
  return d->status_check_period_;
}

void DsProcess::setStatusCheckPeriod(ros::Duration period) noexcept
{
  DS_D(DsProcess);
  d->updateStatusCheckTimer(this, period);
}

boost::shared_ptr<ds_asio::DsConnection> DsProcess::addConnection(const std::string& name,
                                                                  const ds_asio::ReadCallback& callback)
{
  auto nh = nodeHandle();
  DS_D(DsProcess);
  return d->asio_->addConnection(name, callback, nh);
}

boost::shared_ptr<ds_asio::IoSM> DsProcess::addIoSM(const std::string& iosm_name, const std::string& conn_name,
                                                    const ds_asio::ReadCallback& callback)
{
  auto nh = nodeHandle();
  DS_D(DsProcess);
  return d->asio_->addIoSM(iosm_name, conn_name, callback, nh);
}

boost::uuids::uuid DsProcess::uuid() noexcept
{
  const DS_D(DsProcess);
  return d->uuid_;
}

void DsProcess::setup()
{
  DS_D(DsProcess);
  if (d->is_setup_)
  {
    return;
  }

  setupParameters();
  setupConnections();
  setupSubscriptions();
  setupPublishers();
  setupTimers();
  setupServices();

  d->is_setup_ = true;
}

void DsProcess::setupParameters()
{
  const auto health_check_period = ros::param::param<double>("~health_check_period", -1.0);
  if (health_check_period > 0)
  {
    ROS_INFO_STREAM("Setting status updated period to " << health_check_period << " seconds.");
  }
  else
  {
    ROS_INFO_STREAM("Disabling periodic status checks.");
  }
  setStatusCheckPeriod(ros::Duration(health_check_period));
  const auto name_ = ros::param::param<std::string>("~descriptive_name", "NO_NAME_PROVIDED");
  setDescriptiveName(name_);

  DS_D(DsProcess);
  if (ros::param::has("~uuid"))
  {
    d->uuid_ = boost::uuids::string_generator()(ros::param::param<std::string>("~uuid", "0"));
  }
  else
  {
    ROS_WARN_STREAM("No UUID loaded from parameter node.  Using value: " << d->uuid_);
    return;
  }
}

void DsProcess::setupPublishers()
{
  DS_D(DsProcess);
  d->status_publisher_ =
      nodeHandle().advertise<ds_core_msgs::Status>(ros::this_node::getName() + "/status", 10, false);
}

void DsProcess::checkProcessStatus(const ros::TimerEvent& event)
{
  const auto status = statusMessage();
  publishStatus(status);
}

ds_core_msgs::Status DsProcess::statusMessage()
{
  const auto now = ros::Time::now();

  auto status = ds_core_msgs::Status{};
  DS_D(DsProcess);
  status.descriptive_name = d->descriptive_node_name_;

  status.ds_header.io_time = now;
  std::copy(std::begin(d->uuid_.data), std::end(d->uuid_.data), std::begin(status.ds_header.source_uuid));

  // Default to GOOD
  status.status = ds_core_msgs::Status::STATUS_GOOD;

  return status;
}

void DsProcess::publishStatus(const ds_core_msgs::Status& msg)
{
  DS_D(DsProcess);
  d->status_publisher_.publish(msg);
}

void DsProcess::setUuid(const boost::uuids::uuid& uuid) noexcept
{
  DS_D(DsProcess);
  d->uuid_ = uuid;
}
}
