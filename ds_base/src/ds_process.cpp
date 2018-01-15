#include "ds_base/ds_process.h"
#include "ds_base/ds_process_private.h"
#include "ds_core_msgs/Status.h"

namespace ds_base
{


DsProcess::DsProcess()
    : DsProcess(std::unique_ptr<Impl>(new DsProcess::Impl))
{
}

DsProcess::DsProcess(int argc, char** argv, const std::string &name)
    : DsProcess(std::unique_ptr<Impl>(new DsProcess::Impl), argc, argv, name)
{
}

DsProcess::DsProcess(std::unique_ptr<DsProcess::Impl> impl)
  : impl_(std::move(impl))
{
  auto d = d_func();
  d->setup(this);
}

DsProcess::DsProcess(std::unique_ptr<DsProcess::Impl> impl, int argc, char **argv, const std::string &name)
  : impl_(std::move(impl))
{
  ros::init(argc, argv, name);
  auto d = d_func();
  d->setup(this);
}

DsProcess::~DsProcess() = default;


ds_asio::DsNodeHandle* DsProcess::nodeHandle()
{
  auto d = d_func();
  if (!d->node_handle_) {
    d->node_handle_.reset(new ds_asio::DsNodeHandle(&d->asio_->io_service));
  }
  return d->node_handle_.get();
}

void DsProcess::run()
{
  auto d = d_func();
  if (d->status_check_timer_.isValid()){
    d->status_check_timer_.start();
  }
  d->asio_->run();
  d->status_check_timer_.stop();
}


void DsProcess::setDescriptiveName(const std::string &name) noexcept
{
  ROS_INFO_STREAM("Setting descriptive name to: " << name);
  auto d = d_func();
  d->descriptive_node_name_ = name;
}

inline std::string DsProcess::descriptiveName() const noexcept
{
  const auto d = d_func();
  return d->descriptive_node_name_;
}

ros::Duration DsProcess::statusCheckPeriod() const noexcept
{
  const auto d = d_func();
  return d->status_check_period_;
}

void DsProcess::setStatusCheckPeriod(ros::Duration period) noexcept
{
  auto d = d_func();
  d->updateStatusCheckTimer(this, period);
}

boost::shared_ptr<ds_asio::DsConnection> DsProcess::addConnection(const std::string &name, ds_asio::DsAsio::ReadCallback callback)
{
  auto nh = nodeHandle();
  ROS_ASSERT(nh);
  auto d = d_func();
  return d->asio_->addConnection(name, callback, *nh);
}

boost::shared_ptr<ds_asio::DsConnection> DsProcess::connection(const std::string &name) {
  auto d = d_func();
  return d->asio_->connection(name);
}

inline boost::uuids::uuid DsProcess::uuid() const noexcept {
  const auto d = d_func();
  return d->uuid_;
}

}
