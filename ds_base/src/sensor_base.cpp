//
// Created by zac on 12/5/17.
//

#include "ds_base/sensor_base.h"
#include "ds_base/sensor_base_private.h"
#include "ds_base/util.h"
#include "ds_base/StringCommand.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <iterator>

namespace ds_base
{
SensorBase::SensorBase() : SensorBase(std::unique_ptr<SensorBase::Impl>(new SensorBase::Impl()))
{
}

SensorBase::SensorBase(int argc, char* argv[], const std::string& name)
  : SensorBase(std::unique_ptr<SensorBase::Impl>(new SensorBase::Impl()), argc, argv, name)
{
}

SensorBase::SensorBase(std::unique_ptr<SensorBase::Impl> impl) : DsProcess(std::move(impl))
{
}

SensorBase::SensorBase(std::unique_ptr<SensorBase::Impl> impl, int argc, char* argv[], const std::string& name)
  : DsProcess(std::move(impl), argc, argv, name)
{
}

inline auto SensorBase::d_func() noexcept -> Impl*
{
  return static_cast<Impl*>(ds_base::DsProcess::d_func());
}

inline auto SensorBase::d_func() const noexcept -> Impl const*
{
  return static_cast<Impl const*>(ds_base::DsProcess::d_func());
}

SensorBase::~SensorBase() = default;

SensorBase::SensorBase(const SensorBase& rhs)
{
  impl_ = rhs.impl_;
}

void SensorBase::setTimeout(ros::Duration timeout) noexcept
{
  auto d = d_func();
  if (timeout < ros::Duration(0))
  {
    ROS_INFO("Disabling message reception timeout check.");
    d->message_timeout_ = ros::Duration(-1);
  }
  else
  {
    d->message_timeout_ = timeout;
    ROS_INFO_STREAM("Message reception timeout: " << timeout);
  }
}

ros::Duration SensorBase::timeout() const noexcept
{
  const auto d = d_func();
  return d->message_timeout_;
}

void SensorBase::sendCommand(std::string command, std::string connection_)
{
  auto con = connection(connection_);
  if (!con)
  {
    ROS_WARN_STREAM("No connection named '" << connection_ << "'");
    return;
  }

  auto msg = boost::shared_ptr<std::string>(new std::string);
  *msg = std::move(command);
  con->send(std::move(msg));
}

void SensorBase::sendCommand(std::string command, std::string connection, std::string suffix)
{
  if (!suffix.empty() && (!boost::algorithm::ends_with(command, suffix)))
  {
    command.append(suffix);
  }
  sendCommand(std::move(command), std::move(connection));
}

void SensorBase::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  addConnection("instrument", boost::bind(&SensorBase::parseReceivedBytes, this, _1));
}

void SensorBase::setupServices()
{
  DsProcess::setupServices();

  // Need to implement the service callback as a non-capturing lambda (can't cast capturing lambdas
  // to boost::function types).  We'll provide the base pointer with boost::bind next
  auto callback = [](SensorBase* sensor_base, StringCommand::Request& request,
                     StringCommand::Response& response) -> bool {
    sensor_base->sendCommand(request.command, "instrument", "\r\n");
    return true;
  };

  auto nh = nodeHandle();

  auto d = d_func();
  d->send_command_service_ = nh->advertiseService<StringCommand::Request, StringCommand::Response>(
      ros::this_node::getName() + "/send_command", boost::bind<bool>(callback, this, _1, _2));
}

void SensorBase::setupParameters()
{
  ds_base::DsProcess::setupParameters();
  auto d = d_func();
  d->message_timeout_ = ros::Duration(ros::param::param<double>("~message_timeout", 5));

  auto serial_num = ros::param::param<std::string>("~serial_number", "0");
  auto generated_uuid = ds_base::generateUuid(serial_num);

  if (d->uuid_ != generated_uuid)
  {
    ROS_ERROR_STREAM("!!!POTENTIAL CONFIGURATION MISMATCH!!!");
    ROS_ERROR_STREAM("Detected UUID mismatch.");
    ROS_ERROR_STREAM("UUID (param server): " << d->uuid_);
    ROS_ERROR_STREAM("UUID (generated): " << generated_uuid);
  }
  else
  {
    ROS_INFO_STREAM("UUID matches: " << d->uuid_);
  }
}
}
