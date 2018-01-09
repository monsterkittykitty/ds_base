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


ds_asio::DsNodeHandle* DsProcess::getNh()
{
  auto d = d_func();
  if (!d->nh) {
    d->nh.reset(new ds_asio::DsNodeHandle(&d->myAsio->io_service));
  }
  return d->nh.get();
}

void DsProcess::run()
{
  auto d = d_func();
  if (d->status_check_timer_.isValid()){
    d->status_check_timer_.start();
  }
  d->myAsio->run();
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

boost::shared_ptr<ds_asio::DsConnection> DsProcess::addConnection(const std::string &name, boost::function<void(ds_core_msgs::RawData)> callback)
{
  auto nh = getNh();
  ROS_ASSERT(nh);
  auto d = d_func();
  return d->myAsio->addConnection(name, callback, *nh);
}

boost::shared_ptr<ds_asio::DsConnection> DsProcess::connection(const std::string &name) {
  auto d = d_func();
  return d->myAsio->connection(name);
}

ros::Publisher DsProcess::publisher(const std::string& topic, bool *valid) const noexcept
{
  auto const d = d_func();
  auto it = d->publishers_.find(topic);
  if(it == d->publishers_.end()) {
    ROS_ERROR_STREAM("No publisher available for topic name: " << topic);
    if(valid != nullptr) {
      *valid = false;
    }
    return {};
  }

  if (valid != nullptr) {
    *valid = true;
  }

  return it->second;
}

bool DsProcess::hasPublisher(const std::string &name) const noexcept {
  const auto d = d_func();
  return (d->publishers_.find(name) != d->publishers_.end());
}

void DsProcess::_addPublisher(const std::string &name, ros::Publisher pub)
{
  auto d = d_func();
  d->publishers_.insert({name, pub});
}

}
