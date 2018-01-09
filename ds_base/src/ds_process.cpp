#include "ds_base/ds_process.h"
#include "ds_core_msgs/Status.h"

namespace ds_base
{


DsProcess::DsProcess()
  : myAsio(std::unique_ptr<ds_asio::DsAsio>(new ds_asio::DsAsio()))
  , status_check_period_(ros::Duration(-1))
{
}

DsProcess::DsProcess(int argc, char** argv, const std::string &name)
  : myAsio(std::unique_ptr<ds_asio::DsAsio>(new ds_asio::DsAsio(argc, argv, name)))
  , nh(std::unique_ptr<ds_asio::DsNodeHandle>(new ds_asio::DsNodeHandle(&myAsio->io_service)))
  , status_check_period_(ros::Duration(-1))
{
}

DsProcess::~DsProcess() = default;


ds_asio::DsNodeHandle* DsProcess::getNh()
{
  if(!nh)
    {
      nh.reset(new ds_asio::DsNodeHandle(&(myAsio->io_service)));
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

void DsProcess::setup()
{
  setupParameters();
  setupConnections();
  setupSubscriptions();
  setupPublishers();
}


void DsProcess::setupParameters()
{
  const auto health_check_period = ros::param::param<double>("~health_check_period", -1.0);
  if (health_check_period > 0) {
    ROS_INFO_STREAM("Setting status updated period to " << health_check_period << " seconds.");
  }
  else {
    ROS_INFO_STREAM("Disabling periodic status checks.");
  }
  setStatusCheckPeriod(ros::Duration(health_check_period));
  const auto name_ = ros::param::param<std::string>("~descriptive_name", "NO_NAME_PROVIDED");
  setDescriptiveName(name_);
}

void DsProcess::setupPublishers()
{
  addPublisher<ds_core_msgs::Status>("status", 10);
}

void DsProcess::setDescriptiveName(const std::string &name) noexcept
{
  ROS_INFO_STREAM("Setting descriptive name to: " << name);
  descriptive_node_name_ = name;
}

inline std::string DsProcess::descriptiveName() const noexcept
{
  return descriptive_node_name_;
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

boost::shared_ptr<ds_asio::DsConnection> DsProcess::addConnection(const std::string &name, boost::function<void(ds_core_msgs::RawData)> callback)
{
  auto nh = getNh();
  ROS_ASSERT(nh);
  return myAsio->addConnection(name, callback, *nh);
}

boost::shared_ptr<ds_asio::DsConnection> DsProcess::connection(const std::string &name) {
  return myAsio->connection(name);
}

}
