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
}

DsProcess::DsProcess(std::unique_ptr<DsProcess::Impl> impl, int argc, char **argv, const std::string &name)
  : impl_(std::move(impl))
{
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
  if(period == d->status_check_period_) {
    return;
  }

  // Stop pending triggers.
  d->status_check_timer_.stop();

  // Negative durations disable the timer
  if(period < ros::Duration(0)) {
    ROS_INFO_STREAM("Disabling status check timer");
    d->status_check_period_ = ros::Duration(-1);
    return;
  }

  d->status_check_period_ = period;
  // Create a new periodic, auto-starting timer using the nodehandle with our asio callback queue
  d->status_check_timer_ = getNh()->createTimer(d->status_check_period_, &DsProcess::checkProcessStatus, this);
  ROS_INFO_STREAM("Status check timer set to " << d->status_check_period_);
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

template<class T>
void DsProcess::addPublisher(const std::string &topic, uint32_t queue)
{
  auto d = d_func();
  d->addPublisher<T>(this, topic, queue);
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

}
